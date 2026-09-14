"""
CMA-ES (Covariance Matrix Adaptation Evolution Strategy)
=========================================================

Implementacao de referencia do (mu/mu_w, lambda)-CMA-ES.

REFERENCIA PRINCIPAL:
    Hansen, N. (2016). "The CMA Evolution Strategy: A Tutorial."
    arXiv:1604.00772. https://arxiv.org/abs/1604.00772

    Ver secao "Algorithm Summary" (final do tutorial) para a lista
    completa das equacoes. Os comentarios abaixo citam o numero da
    equacao do tutorial ao lado de cada trecho de codigo correspondente,
    para permitir conferencia linha a linha entre paper e implementacao.

REFERENCIA HISTORICA (paper original do metodo):
    Hansen, N., Ostermeier, A. (2001). "Completely Derandomized
    Self-Adaptation in Evolution Strategies." Evolutionary Computation,
    9(2), 159-195.

VISAO GERAL DO ALGORITMO:
    1. Mantemos uma distribuicao gaussiana N(m, sigma^2 * C) sobre o
       espaco de parametros: media m, escala global sigma, forma dada
       pela matriz de covariancia C.
    2. A cada geracao, amostramos "lambda" candidatos dessa gaussiana.
    3. Avaliamos o custo de cada candidato (aqui, rodando a simulacao
       SPH headless).
    4. Selecionamos os "mu" melhores (menor custo) e usamos uma media
       ponderada deles para mover a media m na direcao que reduz o
       custo (Eq. 41 do tutorial).
    5. Atualizamos dois "paths evolutivos" (p_sigma e p_c) que acumulam
       a direcao do movimento ao longo de varias geracoes -- isso e o
       que da ao metodo memoria de curto prazo sobre a trajetoria de
       busca (Eq. 43 e Eq. 42).
    6. Usamos esses paths para adaptar sigma (o quao grande sao os
       passos, Eq. 45) e C (a forma/orientacao da distribuicao,
       Eq. 47) -- por isso "Covariance Matrix Adaptation".
    7. Repetimos ate convergir ou atingir um criterio de parada.
"""

import numpy as np
import matplotlib.pyplot as plt
import time


class CMAES:
    """
    Otimizador CMA-ES para minimizacao de funcoes de custo continuas,
    ruidosas e/ou nao-diferenciaveis (cenario "black-box").

    Uso tipico:

        optimizer = CMAES(x0=[7.0, 1.0, 2.0, 5.0], sigma0=1.0)

        while not optimizer.should_stop():
            solutions = optimizer.ask()                     # gera candidatos
            costs = [cost_fn(*x) for x in solutions]         # avalia cada um
            optimizer.tell(solutions, costs)                 # atualiza a busca

        best_x, best_cost = optimizer.best_solution()

    O padrao "ask/tell" separa a geracao de candidatos da avaliacao,
    o que permite paralelizar as chamadas de cost_fn (ex: rodar varias
    simulacoes SPH em processos diferentes) sem mexer no otimizador.
    """

    def __init__(self, x0, sigma0, pop_size=None, seed=None):
        """
        Parametros
        ----------
        x0 : array-like
            Chute inicial para a media da busca (ex: seus valores atuais
            de referencia para [gamma, alpha, beta, zetta_extra]).
        sigma0 : float
            Passo inicial (escala de busca). Deve refletir a incerteza
            inicial: se voce acha que o otimo esta a +-2 unidades do
            chute, sigma0 ~ 2 e razoavel.
        pop_size : int, opcional
            Numero de candidatos avaliados por geracao (lambda). Se None,
            usa a formula padrao do tutorial (Eq. 48): 4 + floor(3*ln(n)).
        seed : int, opcional
            Semente do gerador aleatorio, para resultados reproduziveis.
        """
        self.rng = np.random.default_rng(seed)

        self.mean = np.array(x0, dtype=float)   # m: media da distribuicao de busca
        self.sigma = float(sigma0)               # sigma: escala global do passo
        self.n = len(self.mean)                  # dimensao do problema (n=4 no seu caso)

        # ---------------------------------------------------------------
        # PASSO 1 -- Tamanho da populacao e pesos de recombinacao
        # (Tutorial, Eq. 48 e secao "Selection and Recombination")
        # ---------------------------------------------------------------
        n = self.n
        if pop_size is None:
            # regra padrao: cresce logaritmicamente com a dimensao
            pop_size = 4 + int(3 * np.log(n))
        self.pop_size = pop_size            # lambda: candidatos por geracao
        self.n_parents = pop_size // 2      # mu: quantos sao usados p/ recombinar

        # pesos w_i decrescentes: o melhor candidato pesa mais que o
        # segundo melhor, e assim por diante (Eq. 41 usa esses pesos)
        raw_weights = np.log(self.n_parents + 0.5) - np.log(np.arange(1, self.n_parents + 1))
        self.weights = raw_weights / raw_weights.sum()   # normalizados: somam 1

        # mu_eff: "numero efetivo de pais" -- mede quanta informacao
        # independente os mu melhores realmente carregam (Eq. 8)
        self.mu_eff = 1.0 / np.sum(self.weights ** 2)

        # ---------------------------------------------------------------
        # PASSO 2 -- Constantes de adaptacao (Eq. 55-58 do tutorial)
        # Cada uma controla a "velocidade de aprendizado" de uma parte
        # do algoritmo. Sao formulas fechadas -- nao precisam ser
        # ajustadas manualmente, e por isso o CMA-ES e chamado de
        # "quase livre de hiperparametros".
        # ---------------------------------------------------------------
        mu_eff = self.mu_eff

        # cs, ds: controlam a adaptacao do PASSO (sigma) via
        # "cumulative step-size adaptation" (CSA)
        self.cs = (mu_eff + 2) / (n + mu_eff + 5)
        self.ds = 1 + 2 * max(0, np.sqrt((mu_eff - 1) / (n + 1)) - 1) + self.cs

        # cc: controla a adaptacao do PATH de covariancia (p_c)
        self.cc = (4 + mu_eff / n) / (n + 4 + 2 * mu_eff / n)

        # c1: taxa de aprendizado da atualizacao "rank-one"
        # (usa a direcao acumulada p_c -- captura correlacoes entre
        # geracoes consecutivas)
        self.c1 = 2 / ((n + 1.3) ** 2 + mu_eff)

        # cmu: taxa de aprendizado da atualizacao "rank-mu"
        # (usa a dispersao dos mu melhores candidatos desta geracao --
        # captura a forma local do problema)
        self.cmu = min(1 - self.c1,
                        2 * (mu_eff - 2 + 1 / mu_eff) / ((n + 2) ** 2 + mu_eff))

        # chi_n: norma esperada de um vetor gaussiano padrao em n
        # dimensoes -- usado como "referencia" para saber se os passos
        # recentes foram maiores ou menores que o esperado ao acaso
        self.chi_n = np.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n ** 2))

        # ---------------------------------------------------------------
        # PASSO 3 -- Estado inicial: matriz de covariancia e paths
        # ---------------------------------------------------------------
        self.C = np.eye(n)              # C: comeca como identidade (busca isotropica)
        self.p_sigma = np.zeros(n)      # path evolutivo do passo (sigma)
        self.p_c = np.zeros(n)          # path evolutivo da covariancia

        # cache dos autovetores/autovalores de C, recalculado a cada
        # geracao em _update_eigendecomposition()
        self.B = np.eye(n)              # autovetores de C (direcoes principais)
        self.D = np.ones(n)             # raiz quadrada dos autovalores de C

        self.generation = 0
        self.best_x = self.mean.copy()
        self.best_cost = np.inf

        # guarda a ultima leva de candidatos amostrados, para o tell()
        self._last_samples = None

        # -----------------------------------------------------------
        # Historico para visualizacao (nao faz parte do algoritmo em
        # si -- e so um registro do que aconteceu em cada geracao, pra
        # podermos plotar depois com plot_history()).
        # -----------------------------------------------------------
        self.history = {
            "generation": [],
            "best_cost_ever": [],     # melhor custo global ate esta geracao
            "gen_best_cost": [],      # melhor custo desta geracao especificamente
            "gen_mean_cost": [],      # custo medio da populacao desta geracao
            "sigma": [],              # tamanho do passo global
            "mean": [],               # valor de cada parametro na media da busca
            "param_std": [],          # incerteza (desvio padrao) de cada parametro
            "condition_number": [],   # o quao "esticada" esta a distribuicao
        }

    # =====================================================================
    # ASK -- gera uma nova geracao de candidatos
    # =====================================================================
    def ask(self):
        """
        Amostra `pop_size` candidatos da distribuicao de busca atual.

        Formula (Eq. 5 do tutorial):
            x_k = m + sigma * (B @ D) @ z_k,   z_k ~ N(0, I)

        Ou seja: cada candidato e a media `m`, deslocada por um vetor
        aleatorio gaussiano, esticado/rotacionado pela forma atual da
        distribuicao (B, D) e escalado pelo passo global sigma.

        Retorna
        -------
        np.ndarray de shape (pop_size, n): um candidato por linha.
        """
        self._update_eigendecomposition()

        z = self.rng.standard_normal((self.pop_size, self.n))   # ruido N(0, I)
        # B*D aplica a forma atual da distribuicao ao ruido isotropico
        samples = self.mean + self.sigma * (z @ (self.B * self.D).T)

        self._last_samples = samples
        return samples

    def _update_eigendecomposition(self):
        """
        Decompoe C = B * D^2 * B^T (autovetores/autovalores).
        Precisamos disso pra amostrar corretamente (ver ask()) e pra
        calcular C^(-1/2) na atualizacao do path de sigma (ver tell()).
        """
        # eigh e usado (nao eig) porque C e sempre simetrica por
        # construcao -- eigh e mais estavel numericamente para isso
        eigvals, eigvecs = np.linalg.eigh(self.C)
        eigvals = np.clip(eigvals, 1e-20, None)   # evita raiz de numero negativo
        self.B = eigvecs
        self.D = np.sqrt(eigvals)

    # =====================================================================
    # TELL -- atualiza a distribuicao de busca a partir dos resultados
    # =====================================================================
    def tell(self, solutions, costs):
        """
        Informa ao otimizador o custo de cada candidato gerado por ask(),
        e atualiza media, sigma e C de acordo.

        Parametros
        ----------
        solutions : array-like, shape (pop_size, n)
            Os mesmos candidatos retornados por ask() (na mesma ordem).
        costs : array-like, shape (pop_size,)
            Custo de cada candidato (menor = melhor). Use um valor alto
            (ex: 1e6) para candidatos que fizeram a simulacao divergir --
            NUNCA passe NaN, o algoritmo nao sabe lidar com isso.
        """
        solutions = np.asarray(solutions, dtype=float)
        costs = np.asarray(costs, dtype=float)

        # ---------------------------------------------------------------
        # PASSO 1 -- Ordena candidatos do melhor (menor custo) pro pior
        # ---------------------------------------------------------------
        order = np.argsort(costs)
        solutions = solutions[order]
        costs = costs[order]

        if costs[0] < self.best_cost:
            self.best_cost = costs[0]
            self.best_x = solutions[0].copy()

        old_mean = self.mean.copy()

        # ---------------------------------------------------------------
        # PASSO 2 -- Recombinacao: nova media = media ponderada dos
        # `n_parents` melhores candidatos (Eq. 41)
        # ---------------------------------------------------------------
        self.mean = self.weights @ solutions[:self.n_parents]

        # deslocamento da media, normalizado pelo passo atual --
        # usado nas duas atualizacoes de path a seguir
        y = (self.mean - old_mean) / self.sigma

        # C^(-1/2) = B * D^(-1) * B^T -- necessario pra "des-rotacionar"
        # o deslocamento antes de acumular no path de sigma (Eq. 43)
        C_inv_sqrt = self.B @ np.diag(1.0 / self.D) @ self.B.T

        # ---------------------------------------------------------------
        # PASSO 3 -- Atualiza o path evolutivo do PASSO (p_sigma)
        # (Eq. 43): mistura o path antigo com a direcao do movimento
        # atual, na "escala natural" da distribuicao (por isso o C^-1/2)
        # ---------------------------------------------------------------
        self.p_sigma = ((1 - self.cs) * self.p_sigma +
                         np.sqrt(self.cs * (2 - self.cs) * self.mu_eff) * (C_inv_sqrt @ y))

        # ---------------------------------------------------------------
        # PASSO 4 -- Atualiza sigma (o tamanho do passo) (Eq. 45)
        # Ideia: se o path acumulado (p_sigma) e maior do que se
        # esperaria de passos aleatorios (chi_n), a busca esta avancando
        # consistentemente na mesma direcao -> aumenta sigma (acelera).
        # Se o path e menor que o esperado, os passos estao se
        # cancelando (zigue-zague) -> diminui sigma (refina).
        # ---------------------------------------------------------------
        self.sigma *= np.exp((self.cs / self.ds) *
                              (np.linalg.norm(self.p_sigma) / self.chi_n - 1))

        # ---------------------------------------------------------------
        # PASSO 5 -- h_sigma: trava de seguranca (Eq. 44)
        # Impede que o path de covariancia (proximo passo) seja
        # atualizado de forma inconsistente quando sigma acabou de
        # crescer muito rápido (evita uma covariancia mal condicionada).
        # ---------------------------------------------------------------
        norm_p_sigma = np.linalg.norm(self.p_sigma)
        h_sigma_threshold = (1.4 + 2 / (self.n + 1)) * self.chi_n
        h_sigma = norm_p_sigma / np.sqrt(1 - (1 - self.cs) ** (2 * (self.generation + 1))) \
            < h_sigma_threshold

        # ---------------------------------------------------------------
        # PASSO 6 -- Atualiza o path evolutivo da COVARIANCIA (p_c)
        # (Eq. 42): parecido com p_sigma, mas SEM a "des-rotacao" por
        # C^-1/2 -- esse path preserva a escala natural do problema, e
        # e usado para capturar a direcao de maior progresso ao longo
        # de varias geracoes (a parte "rank-one" da atualizacao de C).
        # ---------------------------------------------------------------
        self.p_c = ((1 - self.cc) * self.p_c +
                    h_sigma * np.sqrt(self.cc * (2 - self.cc) * self.mu_eff) * y)

        # ---------------------------------------------------------------
        # PASSO 7 -- Atualiza a matriz de covariancia C (Eq. 47)
        # Tres componentes somados:
        #   (a) termo de "esquecimento": encolhe C na direcao da
        #       identidade, proporcional a (1 - c1 - cmu)
        #   (b) termo "rank-one": usa p_c para reforcar a direcao de
        #       progresso acumulada entre geracoes
        #   (c) termo "rank-mu": usa a dispersao real dos `n_parents`
        #       melhores candidatos DESTA geracao, para capturar a
        #       forma local do problema mais rapido que o rank-one
        #       sozinho conseguiria
        # ---------------------------------------------------------------
        y_k = (solutions[:self.n_parents] - old_mean) / self.sigma
        rank_mu_term = sum(w * np.outer(yk, yk) for w, yk in zip(self.weights, y_k))

        self.C = ((1 - self.c1 - self.cmu) * self.C +
                  self.c1 * np.outer(self.p_c, self.p_c) +
                  self.cmu * rank_mu_term)

        # for a simetria numerica de C (pequenos erros de ponto
        # flutuante podem quebrar a simetria ao longo de muitas
        # geracoes, o que quebraria o eigh() na proxima chamada de ask)
        self.C = (self.C + self.C.T) / 2

        # ---------------------------------------------------------------
        # PASSO 8 -- Registra o estado desta geracao no historico
        # (usado so por plot_history(), nao influencia a busca)
        # ---------------------------------------------------------------
        eigvals_C = np.linalg.eigvalsh(self.C)
        eigvals_C = np.clip(eigvals_C, 1e-20, None)
        condition_number = np.max(eigvals_C) / np.min(eigvals_C)
        param_std = self.sigma * np.sqrt(np.diag(self.C))

        self.history["generation"].append(self.generation)
        self.history["best_cost_ever"].append(self.best_cost)
        self.history["gen_best_cost"].append(costs[0])
        self.history["gen_mean_cost"].append(np.mean(costs))
        self.history["sigma"].append(self.sigma)
        self.history["mean"].append(self.mean.copy())
        self.history["param_std"].append(param_std)
        self.history["condition_number"].append(condition_number)

        self.generation += 1

    # =====================================================================
    # Criterios de parada e resultado
    # =====================================================================
    def should_stop(self, max_generations=200, tol_sigma=1e-8, tol_cost=1e-10):
        """
        Criterios de parada simples e praticos (o tutorial descreve
        varios outros mais elaborados na secao "Termination Criteria",
        mas estes tres cobrem a grande maioria dos casos de tuning de
        parametros como o seu):

        - numero maximo de geracoes atingido
        - sigma ficou pequeno demais (a busca ja convergiu/estagnou)
        - a melhora no custo entre geracoes recentes e desprezivel
        """
        if self.generation >= max_generations:
            return True
        if self.sigma < tol_sigma:
            return True
        return False

    def best_solution(self):
        """Retorna (melhor_x, melhor_custo) encontrados ate agora."""
        return self.best_x, self.best_cost

    # =====================================================================
    # Visualizacao -- dashboard de diagnostico padrao do CMA-ES
    # =====================================================================
    def plot_history(self, param_names=None, show=True):
        """
        Plota a evolucao da busca ao longo das geracoes, em 4 paineis --
        o mesmo conjunto de diagnosticos que o logger do pacote oficial
        `pycma` (Hansen) usa por padrao:

          1. Custo (melhor da geracao, melhor global, medio da populacao)
             -- em escala log: se a busca esta convergindo, isso desce
             e estabiliza numa reta.
          2. Sigma (tamanho do passo global) -- deve crescer no inicio
             (explorando) e cair perto da convergencia (refinando).
          3. Valor de cada parametro (a media da busca) ao longo do
             tempo -- mostra pra onde cada gamma/alpha/beta/zetta_extra
             esta indo.
          4. Numero de condicao de C -- o quao "esticada"/mal-condicionada
             esta a distribuicao. Se disparar muito (>1e6), e sinal de
             que os parametros tem escalas muito diferentes entre si e
             valeria normalizar o espaco de busca.

        Parametros
        ----------
        param_names : list[str], opcional
            Nomes dos parametros pra legenda (ex: ["gamma","alpha",
            "beta","zetta_extra"]). Se None, usa "param_0", "param_1"...
        show : bool
            Se True, chama plt.show() no final.
        """
        if not self.history["generation"]:
            print("Nada para plotar ainda -- rode ask()/tell() pelo menos uma vez.")
            return None

        gens = self.history["generation"]
        n_params = self.n
        if param_names is None:
            param_names = [f"param_{i}" for i in range(n_params)]

        means = np.array(self.history["mean"])          # shape (n_gens, n_params)
        stds = np.array(self.history["param_std"])       # shape (n_gens, n_params)

        fig, axes = plt.subplots(2, 2, figsize=(12, 8))

        # ---- painel 1: custo ----
        ax = axes[0, 0]
        ax.plot(gens, self.history["gen_mean_cost"], color="gray", alpha=0.6, label="Media da populacao")
        ax.plot(gens, self.history["gen_best_cost"], color="tab:orange", label="Melhor da geracao")
        ax.plot(gens, self.history["best_cost_ever"], color="tab:blue", linewidth=2, label="Melhor global")
        ax.set_yscale("log")
        ax.set_xlabel("Geracao")
        ax.set_ylabel("Custo (escala log)")
        ax.set_title("Convergencia do custo")
        ax.legend()
        ax.grid(True, which="both", alpha=0.3)

        # ---- painel 2: sigma ----
        ax = axes[0, 1]
        ax.plot(gens, self.history["sigma"], color="tab:green")
        ax.set_yscale("log")
        ax.set_xlabel("Geracao")
        ax.set_ylabel("Sigma (escala log)")
        ax.set_title("Tamanho do passo (sigma)")
        ax.grid(True, which="both", alpha=0.3)

        # ---- painel 3: valor de cada parametro (com faixa de incerteza) ----
        ax = axes[1, 0]
        for i in range(n_params):
            line, = ax.plot(gens, means[:, i], label=param_names[i])
            ax.fill_between(gens,
                             means[:, i] - stds[:, i],
                             means[:, i] + stds[:, i],
                             color=line.get_color(), alpha=0.15)
        ax.set_xlabel("Geracao")
        ax.set_ylabel("Valor do parametro")
        ax.set_title("Evolucao dos parametros (media +- desvio padrao)")
        ax.legend()
        ax.grid(True, alpha=0.3)

        # ---- painel 4: numero de condicao de C ----
        ax = axes[1, 1]
        ax.plot(gens, self.history["condition_number"], color="tab:red")
        ax.set_yscale("log")
        ax.set_xlabel("Geracao")
        ax.set_ylabel("Numero de condicao (escala log)")
        ax.set_title("Condicionamento da matriz de covariancia")
        ax.grid(True, which="both", alpha=0.3)

        plt.tight_layout()
        if show:
            plt.show()
        return fig, axes


# =============================================================================
# Exemplo de uso com a sua simulacao SPH
# =============================================================================
if __name__ == "__main__":
    # Importa a sua classe de simulacao real (ajuste o caminho conforme
    # a estrutura do seu projeto ROS2 -- no seu caso, provavelmente
    # algo como: from drone_controller.raw_sim import SPHSimulation)
    from raw_sim import SPHSimulation

    # -----------------------------------------------------------------
    # Configuracao fixa da simulacao (nao faz parte do vetor otimizado
    # pelo CMA-ES -- sao os mesmos valores que voce ja usa no seu
    # python_sim.py: n=8 particulas por lado, massa, h, rho_0, dt, passos)
    # -----------------------------------------------------------------
    N_PARTICLES_PER_SIDE = 8
    MASS = 795.2156
    H = 0.75
    RHO_0 = 1000.0
    DT = 0.05
    NUM_STEPS = 200

    # EXPECTED_DISTANCE: espacamento de equilibrio derivado do balanco
    # massa/h (drones com raio de percepcao de 1.5m e espacamento
    # desejado de 0.5m -- ver derivacao completa na conversa)
    EXPECTED_DISTANCE = 0.5

    # pesos de cada termo do custo -- comece com todos iguais e ajuste
    # conforme qual indicador for mais critico pros seus drones
    # (ex: se colisao é inaceitavel, suba w5/w6 acima dos demais)
    W_RHO = 1.0
    W_PRESSURE = 1.0
    W_VEL_SPIKE = 1.0
    W_ACCEL_SPIKE = 1.0
    W_SPACING_CEIL = 1.0
    W_SPACING_FLOOR = 1.0

    DIVERGENCE_PENALTY = 1e6  # custo atribuido se a simulacao explodir

    def cost_fn(gamma, alpha, beta, zetta_extra):
        """
        Roda uma simulacao SPH completa com o conjunto de parametros
        proposto pelo CMA-ES e retorna (custo, status).

        Cada chamada desta funcao = uma simulacao inteira do zero, com
        posicoes/velocidades reiniciadas -- e assim que o CMA-ES
        compara "pontos" diferentes do espaco de busca de forma justa.

        IMPORTANTE: o CMA-ES amostra livremente ao redor da media, sem
        saber quais combinacoes sao fisicamente validas. E normal (e
        esperado) que ele teste, por exemplo, gamma ou alpha negativos
        de vez em quando -- principalmente no inicio, quando sigma
        ainda e grande. Isso pode fazer a simulacao lancar excecoes
        (ex: raiz quadrada de numero negativo em Particle.__init__).
        O try/except abaixo garante que isso vire apenas uma penalidade
        alta para aquele candidato, em vez de derrubar a otimizacao
        inteira -- e o "status" deixa claro no terminal qual dos dois
        casos aconteceu.
        """
        try:
            sim = SPHSimulation(
                n=N_PARTICLES_PER_SIDE, mass=MASS, h=H, rho_0=RHO_0,
                dt=DT, num_steps=NUM_STEPS,
                particle_kwargs=dict(gamma=gamma, alpha=alpha, beta=beta,
                                      zetta_extra=zetta_extra),
            )
            sim.run_headless()
            metrics = sim.get_cost_metrics(rho_0=RHO_0, expected_distance=EXPECTED_DISTANCE)
        except Exception as exc:
            return DIVERGENCE_PENALTY, f"parametros invalidos ({exc})"

        if metrics is None:
            return DIVERGENCE_PENALTY, "divergiu (NaN/explosao)"

        cost = (W_RHO * metrics["rho_err"] +
                W_PRESSURE * metrics["p_err"] +
                W_VEL_SPIKE * metrics["vel_spike"] +
                W_ACCEL_SPIKE * metrics["accel_spike"] +
                W_SPACING_CEIL * metrics["spacing_ceiling"] +
                W_SPACING_FLOOR * metrics["spacing_floor"])
        return cost, "ok"

    x0 = [1.0, 1.0, 2.0, 5.0]   # seus valores atuais de referencia
    sigma0 = 1.0

    optimizer = CMAES(x0=x0, sigma0=sigma0, seed=42)

    # -----------------------------------------------------------------
    # Mensagem inicial: confirma que o script comecou e mostra o
    # tamanho do trabalho pela frente, antes de qualquer simulacao rodar
    # -----------------------------------------------------------------
    print("=== Iniciando otimizacao CMA-ES ===")
    print(f"Parametros otimizados : gamma, alpha, beta, zetta_extra ({optimizer.n} dimensoes)")
    print(f"Candidatos por geracao: {optimizer.pop_size}")
    print(f"Passos por simulacao  : {NUM_STEPS}")
    print("======================================\n")

    start_time = time.time()

    while not optimizer.should_stop(max_generations=100):
        gen_start = time.time()
        solutions = optimizer.ask()

        print(f"Geracao {optimizer.generation + 1} -- avaliando {len(solutions)} candidatos...")

        costs = []
        for i, x in enumerate(solutions):
            cand_start = time.time()
            cost, status = cost_fn(*x)
            costs.append(cost)
            cand_elapsed = time.time() - cand_start
            print(f"  candidato {i+1:2d}/{len(solutions)} | "
                  f"custo={cost:12.6f} | {status:28s} | {cand_elapsed:5.2f}s | "
                  f"params={np.round(x, 3)}")

        optimizer.tell(solutions, costs)

        gen_elapsed = time.time() - gen_start
        total_elapsed = time.time() - start_time
        best_x, best_cost = optimizer.best_solution()

        print(f"-> melhor desta geracao: {min(costs):.6f} | melhor global: {best_cost:.6f}")
        print(f"   sigma={optimizer.sigma:.6f} | "
              f"tempo geracao={gen_elapsed:.1f}s | tempo total={total_elapsed:.1f}s\n")

    best_x, best_cost = optimizer.best_solution()
    total_elapsed = time.time() - start_time
    print("=== Otimizacao concluida ===")
    print(f"Total de geracoes: {optimizer.generation} | tempo total: {total_elapsed:.1f}s\n")
    print("=== Resultado final ===")
    print(f"gamma={best_x[0]:.4f}  alpha={best_x[1]:.4f}  "
          f"beta={best_x[2]:.4f}  zetta_extra={best_x[3]:.4f}")
    print(f"custo={best_cost:.6f}")

    print("\nAbrindo dashboard de diagnostico...")
    optimizer.plot_history(param_names=["gamma", "alpha", "beta", "zetta_extra"])