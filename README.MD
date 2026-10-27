# SPH_Project

Controlador SPH para enxames de Crazyflies (pacote ROS 2 `drone_controller`). Roda no container `crazysim`,
junto com o CrazySim (Gazebo + firmware) e o Crazyswarm2. Os comandos abaixo usam só o `ros2 launch`.

## 1. Instalação e preparação

```bash
cd ~/SPH_Project
docker compose build crazysim            # uma vez, 10-20 min
docker compose up -d crazysim
```

Para o CrazySim, após cada reinicialização do PC, deixe o processador em `performance` (fator de tempo
real ≥ 0,9):

```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

## 2. Terminal do container

Todo terminal começa assim:

```bash
cd ~/SPH_Project && docker compose exec crazysim bash
source /opt/ros/humble/setup.bash
source /CrazySim/crazyswarm2_ws/install/setup.bash
```

**Antes de rodar o controlador, compile o pacote.** Isso é obrigatório na primeira vez, sempre que o
container for recriado e depois de editar `src/`. Na dúvida, compile de novo. Faça isso no terminal do controlador:

```bash
cd /root/dev_ws && colcon build --symlink-install
source /root/dev_ws/src/dev_setup.bash
```

Nos outros terminais, que não compilam, basta carregar o pacote já compilado:

```bash
source /root/dev_ws/src/dev_setup.bash
```

No terminal do controlador, defina a função `sph` (lança o controlador com os parâmetros comuns em `$BASE`):

```bash
sph() { L=$1; shift; ros2 launch drone_controller plain_sph.launch.py $BASE "$@" run_label:=$L run_id:=$(date +%Y%m%d_%H%M%S)_$L; }
COMUM="rho_0:=1000.0 alpha:=1.0 beta:=2.0 f_g1:=30.0 f_beta:=1.0 sph_gain:=0.5 control_period:=0.05 dt_model:=0.0 fixed_z:=0.5 max_velocity:=0.2 max_vz:=0.2 z_stretch:=1.0 test_profile:=none obstacles:=none obs_sigma:=10"
Z2="z_min:=0.25 z_max:=2.5 z_margin:=0.3 z_wall_k:=2.0"     # limites verticais padrão
Z3="z_min:=0.2 z_max:=1.5 z_margin:=0.1 z_wall_k:=8"        # limites da arena real
```

Quando um teste define `obstacles`, `max_velocity` ou `max_vz`, o valor do teste prevalece sobre o de `$COMUM`.

**Encerrar um ensaio:**
1. Em outro terminal do container, pouse:
```bash
ros2 topic pub -w 1 -t 5 /sph/land std_msgs/msg/Empty '{}'
```
2. Dê `Ctrl+C` no `sph`, que grava `src/logs/runs/<run_id>/`.
3. Gere os gráficos no host (seção 6).

Pouso de emergência, pelo firmware:
```bash
ros2 service call /all/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 3, nanosec: 0}}"
```

## 3. CrazySim

**Passos de cada ensaio:**

1. **Host:** gere a lista de veículos (N e r no comentário de cada ensaio) e reinicie o container.
```bash
python3 gen_config.py --n <N> --radius <r> --config-dir config && docker compose restart -t 1 crazysim
```
2. **Terminal 1:** Gazebo e firmware.
```bash
rm -rf /tmp/gz* /tmp/sitl* && cd /CrazySim/crazyflie-firmware && bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_headless.sh -f agents.txt
```
3. **Terminal 2**, quando os veículos aparecerem no terminal 1: Crazyswarm2.
```bash
ros2 launch crazyflie launch.py backend:=cflib rviz:=False
```
4. **Terminal 3:** compile o pacote e defina `sph`/`COMUM` (seção 2); depois decole, defina `BASE` e rode a
   linha `sph` do ensaio.
```bash
ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff '{height: 0.5, duration: {sec: 2, nanosec: 0}}'
BASE="$COMUM platform:=sim sim_clock:=gz start_gate:=flight stale_policy:=predict state_timeout_s:=0"
```

### 3.1 Arena do laboratório (−1,2 a 1,2 m)

Cabem no quadrado do laboratório.

```bash
# Conformação: circular — gen_config --n 5 --radius 0.255
sph arena_circ_5 total_robots:=5 dim:=2 potential:=default particle_mass:=437.687 smoothing_h:=0.5 gamma:=3 zetta_extra:=50 xc:=0.4 yc:=0.4 zc:=1 R:=0.51 f_k:=10 f_kt:=0 $Z2
# Conformação: esférico — gen_config --n 5 --radius 0.255
sph arena_esfera_5 total_robots:=5 dim:=3 potential:=default particle_mass:=81.0868 smoothing_h:=0.3 gamma:=7 zetta_extra:=40 xc:=0.4 yc:=0.4 zc:=1.0 R:=0.337 f_k:=2 f_kt:=0 $Z3
# Conformação: quártico no plano — gen_config --n 5 --radius 0.300
sph arena_quart2d_5 total_robots:=5 dim:=2 potential:=quartic particle_mass:=437.687 smoothing_h:=0.5 gamma:=3 zetta_extra:=50 xc:=0 yc:=0 zc:=1 R:=0.035 f_k:=20 f_kt:=0 $Z2
# Conformação: quártico no espaço — gen_config --n 5 --radius 0.425
sph arena_quart3d_5 total_robots:=5 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=40 xc:=0.7 yc:=0.7 zc:=1 R:=0.035 f_k:=60 f_kt:=0 $Z3
# Circulação: plano — gen_config --n 5 --radius 0.300
sph arena_quart2d_tang_5 total_robots:=5 dim:=2 potential:=quartic particle_mass:=437.687 smoothing_h:=0.5 gamma:=3 zetta_extra:=50 xc:=0 yc:=0 zc:=1 R:=0.035 f_k:=20 f_kt:=3 $Z2
# Circulação: espaço — gen_config --n 5 --radius 0.425
sph arena_quart3d_tang_5 total_robots:=5 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=40 xc:=0.65 yc:=0.65 zc:=1 R:=0.035 f_k:=60 f_kt:=1 $Z3
# Obstáculos: passagem estreitada — gen_config --n 4 --radius 0.212
sph arena_obs_portao_4 total_robots:=4 dim:=2 potential:=default particle_mass:=133.183 smoothing_h:=0.25 gamma:=3 zetta_extra:=50 xc:=0.85 yc:=0.85 zc:=1 R:=0.283 f_k:=10 f_kt:=0 obstacles:=0.212,0.778/0.778,0.212 obs_sigma:=2 $Z2
# Obstáculos: curva com circulação — gen_config --n 5 --radius 0.500
sph arena_obs_quart2d_tang_5 total_robots:=5 dim:=2 potential:=quartic particle_mass:=194.322 smoothing_h:=0.3 gamma:=3 zetta_extra:=50 xc:=0 yc:=0 zc:=1 R:=0.4 f_k:=15 f_kt:=7 obstacles:=-0.76,0 obs_sigma:=2 $Z2
```

### 3.2 Ensaios da monografia com até 5 veículos

Mesma geometria da monografia (alvo afastado da decolagem), sem limite de arena.

```bash
# Conformação: circular — gen_config --n 5 --radius 0.255
sph cs_circ_5 total_robots:=5 dim:=2 potential:=default particle_mass:=129.541 smoothing_h:=0.3 gamma:=3 zetta_extra:=50 xc:=2 yc:=2 zc:=1 R:=0.255 f_k:=10 f_kt:=0 $Z2
# Conformação: esférico — gen_config --n 5 --radius 0.408
sph cs_esfera_5 total_robots:=5 dim:=3 potential:=default particle_mass:=81.0868 smoothing_h:=0.3 gamma:=7 zetta_extra:=40 xc:=2 yc:=2 zc:=1.2 R:=0.337 f_k:=2 f_kt:=0 $Z2
# Conformação: quártico no plano — gen_config --n 5 --radius 0.510
sph cs_quart2d_5 total_robots:=5 dim:=2 potential:=quartic particle_mass:=437.687 smoothing_h:=0.5 gamma:=3 zetta_extra:=50 xc:=2 yc:=2 zc:=1 R:=0.035 f_k:=20 f_kt:=0 $Z2
# Conformação: quártico no espaço — gen_config --n 5 --radius 0.425
sph cs_quart3d_5 total_robots:=5 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=40 xc:=1.5 yc:=1.5 zc:=1 R:=0.035 f_k:=60 f_kt:=0 $Z2
# Circulação: plano — gen_config --n 5 --radius 0.510
sph cs_quart2d_tang_5 total_robots:=5 dim:=2 potential:=quartic particle_mass:=437.687 smoothing_h:=0.5 gamma:=3 zetta_extra:=50 xc:=2 yc:=2 zc:=1 R:=0.035 f_k:=20 f_kt:=3 $Z2
# Circulação: espaço — gen_config --n 5 --radius 0.425
sph cs_quart3d_tang_5 total_robots:=5 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=40 xc:=1.5 yc:=1.5 zc:=1 R:=0.035 f_k:=60 f_kt:=1 $Z2
# Obstáculos: passagem estreitada — gen_config --n 4 --radius 0.354
sph cs_obs_circ_portao_4 total_robots:=4 dim:=2 potential:=default particle_mass:=194.322 smoothing_h:=0.3 gamma:=3 zetta_extra:=50 xc:=2 yc:=2 zc:=1 R:=0.354 f_k:=10 f_kt:=0 obstacles:=0.65,1.35/1.35,0.65 obs_sigma:=2 $Z2
# Obstáculos: curva com circulação — gen_config --n 3 --radius 0.289
sph cs_obs_quart2d_tang_3 total_robots:=3 dim:=2 potential:=quartic particle_mass:=194.322 smoothing_h:=0.3 gamma:=3 zetta_extra:=50 xc:=2 yc:=2 zc:=1 R:=0.4 f_k:=15 f_kt:=7 obstacles:=2.0,2.76 obs_sigma:=2 $Z2
```

### 3.3 Testes reais no CrazySim (2 a 5 veículos)

Os mesmos testes da seção 5, para ensaiar no CrazySim antes de voar. Em cada teste, a primeira linha roda
no host e a segunda no terminal 3. Entre as duas, abra de novo os terminais do container (o reinício os
fecha), suba o Gazebo e o Crazyswarm2 (passos 2 e 3), redefina as variáveis da seção 2 e decole. No terminal
3, use:

```bash
BASE="$COMUM platform:=sim sim_clock:=gz start_gate:=flight stale_policy:=predict state_timeout_s:=0"
```

**Conformação: circular**

```bash
python3 gen_config.py --n 2 --radius 0.300 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_circ_2 total_robots:=2 dim:=2 potential:=default particle_mass:=487.392 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.3 f_k:=2 f_kt:=0 $Z2

python3 gen_config.py --n 3 --radius 0.346 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_circ_3 total_robots:=3 dim:=2 potential:=default particle_mass:=437.722 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.346 f_k:=2 f_kt:=0 $Z2

python3 gen_config.py --n 4 --radius 0.424 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_circ_4 total_robots:=4 dim:=2 potential:=default particle_mass:=435.313 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.424 f_k:=2 f_kt:=0 $Z2

python3 gen_config.py --n 5 --radius 0.510 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_circ_5 total_robots:=5 dim:=2 potential:=default particle_mass:=437.687 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.51 f_k:=2 f_kt:=0 $Z2

```

**Conformação: quártico no plano**

```bash
python3 gen_config.py --n 2 --radius 0.164 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart2d_2 total_robots:=2 dim:=2 potential:=quartic particle_mass:=487.392 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0009 f_k:=320 f_kt:=0 $Z2

python3 gen_config.py --n 3 --radius 0.245 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart2d_3 total_robots:=3 dim:=2 potential:=quartic particle_mass:=437.722 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0045 f_k:=28 f_kt:=0 $Z2

python3 gen_config.py --n 4 --radius 0.280 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart2d_4 total_robots:=4 dim:=2 potential:=quartic particle_mass:=435.313 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0142 f_k:=5 f_kt:=0 $Z2

python3 gen_config.py --n 5 --radius 0.400 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart2d_5 total_robots:=5 dim:=2 potential:=quartic particle_mass:=437.687 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.035 f_k:=2 f_kt:=0 $Z2

```

**Conformação: quártico no espaço**

```bash
python3 gen_config.py --n 2 --radius 0.300 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart3d_2 total_robots:=2 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0009 f_k:=320 f_kt:=0 $Z3

python3 gen_config.py --n 3 --radius 0.300 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart3d_3 total_robots:=3 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0045 f_k:=28 f_kt:=0 $Z3

python3 gen_config.py --n 4 --radius 0.354 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart3d_4 total_robots:=4 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0142 f_k:=5 f_kt:=0 $Z3

python3 gen_config.py --n 5 --radius 0.425 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart3d_5 total_robots:=5 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.035 f_k:=2 f_kt:=0 $Z3

```

**Circulação: plano**

```bash
python3 gen_config.py --n 2 --radius 0.164 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart2d_tang_2 total_robots:=2 dim:=2 potential:=quartic particle_mass:=487.392 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0009 f_k:=320 f_kt:=47 $Z2

python3 gen_config.py --n 3 --radius 0.245 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart2d_tang_3 total_robots:=3 dim:=2 potential:=quartic particle_mass:=437.722 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0045 f_k:=28 f_kt:=14 $Z2

python3 gen_config.py --n 4 --radius 0.280 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart2d_tang_4 total_robots:=4 dim:=2 potential:=quartic particle_mass:=435.313 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0142 f_k:=5 f_kt:=6 $Z2

python3 gen_config.py --n 5 --radius 0.400 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart2d_tang_5 total_robots:=5 dim:=2 potential:=quartic particle_mass:=437.687 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.035 f_k:=2 f_kt:=3 $Z2

```

**Circulação: espaço**

```bash
python3 gen_config.py --n 2 --radius 0.300 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart3d_tang_2 total_robots:=2 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0009 f_k:=320 f_kt:=16 $Z3

python3 gen_config.py --n 3 --radius 0.300 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart3d_tang_3 total_robots:=3 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0045 f_k:=28 f_kt:=5 $Z3

python3 gen_config.py --n 4 --radius 0.354 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart3d_tang_4 total_robots:=4 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0142 f_k:=5 f_kt:=2 $Z3

python3 gen_config.py --n 5 --radius 0.425 --config-dir config && docker compose restart -t 1 crazysim
sph cs_real_quart3d_tang_5 total_robots:=5 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.035 f_k:=5 f_kt:=1 $Z3

```

## 4. Dinâmica simplificada

Sem Gazebo nem Crazyswarm2 e sem decolagem. São dois terminais do container (seção 2). No A, compile o
pacote, defina `sph`/`COMUM` e rode o `sph`. No B, rode o simulador de partículas depois que o controlador
subir.

```bash
# terminal A
BASE="$COMUM $Z2 platform:=sim sim_clock:=wall start_gate:=all_known stale_policy:=predict state_timeout_s:=0"
# terminal B
part() { ros2 run drone_controller particle_sim --ros-args -p fixed_z:=0.5 -p tau:=0.15 -p delay:=0.1 -p tau_z:=0.3 -p max_vz:=0.2 -p z_drift:=0.0 -p viz:=rviz -p viz_hz:=5.0 "$@"; }
```

Ensaios da monografia (linha `sph` no terminal A, linha `part` no B):

```bash
# Conformação: circular, 20
sph p_circ_20 total_robots:=20 dim:=2 potential:=default particle_mass:=131.946 smoothing_h:=0.3 gamma:=3 zetta_extra:=50 xc:=2 yc:=2 zc:=1 R:=0.96 f_k:=10 f_kt:=0
part -p total_robots:=20 -p initial_positions:=-0.6,0.45/-0.3,0.45/0,0.45/0.3,0.45/0.6,0.45/-0.6,0.15/-0.3,0.15/0,0.15/0.3,0.15/0.6,0.15/-0.6,-0.15/-0.3,-0.15/0,-0.15/0.3,-0.15/0.6,-0.15/-0.6,-0.45/-0.3,-0.45/0,-0.45/0.3,-0.45/0.6,-0.45

# Conformação: esférico, 8
sph p_esfera total_robots:=8 dim:=3 potential:=default particle_mass:=83.5371 smoothing_h:=0.3 gamma:=7 zetta_extra:=40 xc:=2 yc:=2 zc:=1.2 R:=0.4544 f_k:=2 f_kt:=0 max_velocity:=0.5 max_vz:=0.25
part -p total_robots:=8 -p initial_positions:=-0.5,0.5/0,0.5/0.5,0.5/-0.5,0/0,0/0.5,0/-0.25,-0.5/0.25,-0.5 -p z_jitter:=0.12 -p max_vz:=0.25

# Conformação: quártico no plano, 12
sph p_quart2d total_robots:=12 dim:=2 potential:=quartic particle_mass:=172.382 smoothing_h:=0.3 gamma:=3 zetta_extra:=50 xc:=2 yc:=2 zc:=1 R:=0.23 f_k:=10 f_kt:=0
part -p total_robots:=12 -p initial_positions:=-0.6,0.4/-0.2,0.4/0.2,0.4/0.6,0.4/-0.6,0/-0.2,0/0.2,0/0.6,0/-0.6,-0.4/-0.2,-0.4/0.2,-0.4/0.6,-0.4

# Conformação: quártico no espaço, 8
sph p_quart3d total_robots:=8 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=40 xc:=1.5 yc:=1.5 zc:=1 R:=0.23 f_k:=10 f_kt:=0
part -p total_robots:=8 -p initial_positions:=-0.5,0.5/0,0.5/0.5,0.5/-0.5,0/0,0/0.5,0/-0.25,-0.5/0.25,-0.5 -p z_jitter:=0.12

# Circulação: plano, 12
sph p_quart2d_tang total_robots:=12 dim:=2 potential:=quartic particle_mass:=172.382 smoothing_h:=0.3 gamma:=3 zetta_extra:=50 xc:=2 yc:=2 zc:=1 R:=0.32 f_k:=7 f_kt:=2
part -p total_robots:=12 -p initial_positions:=-0.6,0.4/-0.2,0.4/0.2,0.4/0.6,0.4/-0.6,0/-0.2,0/0.2,0/0.6,0/-0.6,-0.4/-0.2,-0.4/0.2,-0.4/0.6,-0.4

# Circulação: espaço, 8
sph p_quart3d_tang total_robots:=8 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=40 xc:=1.5 yc:=1.5 zc:=1 R:=0.23 f_k:=10 f_kt:=1
part -p total_robots:=8 -p initial_positions:=-0.5,0.5/0,0.5/0.5,0.5/-0.5,0/0,0/0.5,0/-0.25,-0.5/0.25,-0.5 -p z_jitter:=0.12

# Obstáculos: passagem estreitada, 14
sph p_obs_circ_portao_14 total_robots:=14 dim:=2 potential:=default particle_mass:=194.322 smoothing_h:=0.3 gamma:=3 zetta_extra:=50 xc:=2 yc:=2 zc:=1 R:=1.123 f_k:=10 f_kt:=0 obstacles:=0.65,1.35/1.35,0.65 obs_sigma:=2
part -p total_robots:=14 -p initial_positions:=-0.75,0.75/-0.25,0.75/0.25,0.75/0.75,0.75/-0.75,0.25/-0.25,0.25/0.25,0.25/0.75,0.25/-0.75,-0.25/-0.25,-0.25/0.25,-0.25/0.75,-0.25/-0.25,-0.75/0.25,-0.75

# Obstáculos: curva com circulação, 15
sph p_obs_quart2d_tang_15 total_robots:=15 dim:=2 potential:=quartic particle_mass:=194.322 smoothing_h:=0.3 gamma:=3 zetta_extra:=50 xc:=3.8 yc:=3.8 zc:=1 R:=6.866 f_k:=3 f_kt:=7 obstacles:=3.8,5.14 obs_sigma:=2
part -p total_robots:=15 -p initial_positions:=-0.75,0.75/-0.25,0.75/0.25,0.75/0.75,0.75/-0.75,0.25/-0.25,0.25/0.25,0.25/0.75,0.25/-0.75,-0.25/-0.25,-0.25/0.25,-0.25/0.75,-0.25/-0.5,-0.75/0,-0.75/0.5,-0.75
```

## 5. Drones reais

Arena de −1,2 a 1,2 m; |v| e |v_z| ≤ 0,2 m/s; decolagem a 0,5 m.

**Preparação (host, uma vez):** habilite em `config/crazyflies_real.yaml` exatamente os N drones do teste
(`cf_0 .. cf_{N-1}`) e rode:

```bash
cp config/crazyflies_real.yaml config/crazyflies.yaml
docker compose up -d crazysim && docker compose restart -t 1 crazysim
```

Todos os potenciais são centrados em (0,6; 0,6) m.

**Posições de decolagem** (x, y em m, ±5 cm; confira com `ros2 topic echo --once /cf_0/state`):

| Teste | cf_0 | cf_1 | cf_2 | cf_3 | cf_4 |
|---|---|---|---|---|---|
| circ 2 | (+0.88, +0.71) | (+0.32, +0.49) |  |  |  |
| circ 3 | (+0.92, +0.73) | (+0.32, +0.81) | (+0.56, +0.26) |  |  |
| circ 4 | (+0.99, +0.76) | (+0.44, +0.99) | (+0.21, +0.44) | (+0.76, +0.21) |  |
| circ 5 | (+1.07, +0.79) | (+0.56, +1.11) | (+0.10, +0.72) | (+0.33, +0.17) | (+0.93, +0.21) |
| quártico (2D e 3D, com e sem circulação) 2 | (+0.73, +0.65) | (+0.47, +0.55) |  |  |  |
| quártico (2D e 3D, com e sem circulação) 3 | (+0.79, +0.68) | (+0.38, +0.81) | (+0.53, +0.41) |  |  |
| quártico (2D e 3D, com e sem circulação) 4 | (+0.86, +0.71) | (+0.49, +0.86) | (+0.34, +0.49) | (+0.71, +0.34) |  |
| quártico (2D e 3D, com e sem circulação) 5 | (+0.93, +0.74) | (+0.61, +0.91) | (+0.27, +0.76) | (+0.30, +0.23) | (+0.88, +0.23) |

**Terminal 1:** Crazyswarm2.
```bash
ros2 launch crazyflie launch.py backend:=cflib rviz:=False
```

**Testes de movimento** (terminal 2; decolam e pousam sozinhos; comece com 1 drone; `Ctrl+C` pousa):

```bash
cd /root/dev_ws/src/drone_controller/testes
python3 frente_tras.py --v 0.2 --tempo 2
python3 frente_tras_lados.py --v 0.2 --tempo 2
```

**Testes SPH** (terminal 2): compile o pacote e defina `sph`/`COMUM` (seção 2), decole e rode a linha do teste. O controlador só age com todos acima de
0,4 m. Para pousar, use o comando de pouso da seção 2 em outro terminal.

```bash
ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff '{height: 0.5, duration: {sec: 3, nanosec: 0}}'
BASE="$COMUM platform:=real sim_clock:=wall start_gate:=flight stale_policy:=hold state_timeout_s:=1.0"

# Conformação: circular
sph real_circ_2 total_robots:=2 dim:=2 potential:=default particle_mass:=487.392 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.3 f_k:=2 f_kt:=0 $Z2
sph real_circ_3 total_robots:=3 dim:=2 potential:=default particle_mass:=437.722 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.346 f_k:=2 f_kt:=0 $Z2
sph real_circ_4 total_robots:=4 dim:=2 potential:=default particle_mass:=435.313 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.424 f_k:=2 f_kt:=0 $Z2
sph real_circ_5 total_robots:=5 dim:=2 potential:=default particle_mass:=437.687 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.51 f_k:=2 f_kt:=0 $Z2
# Conformação: quártico no plano
sph real_quart2d_2 total_robots:=2 dim:=2 potential:=quartic particle_mass:=487.392 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0009 f_k:=320 f_kt:=0 $Z2
sph real_quart2d_3 total_robots:=3 dim:=2 potential:=quartic particle_mass:=437.722 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0045 f_k:=28 f_kt:=0 $Z2
sph real_quart2d_4 total_robots:=4 dim:=2 potential:=quartic particle_mass:=435.313 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0142 f_k:=5 f_kt:=0 $Z2
sph real_quart2d_5 total_robots:=5 dim:=2 potential:=quartic particle_mass:=437.687 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.035 f_k:=2 f_kt:=0 $Z2
# Conformação: quártico no espaço
sph real_quart3d_2 total_robots:=2 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0009 f_k:=320 f_kt:=0 $Z3
sph real_quart3d_3 total_robots:=3 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0045 f_k:=28 f_kt:=0 $Z3
sph real_quart3d_4 total_robots:=4 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0142 f_k:=5 f_kt:=0 $Z3
sph real_quart3d_5 total_robots:=5 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.035 f_k:=2 f_kt:=0 $Z3
# Circulação: plano
sph real_quart2d_tang_2 total_robots:=2 dim:=2 potential:=quartic particle_mass:=487.392 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0009 f_k:=320 f_kt:=47 $Z2
sph real_quart2d_tang_3 total_robots:=3 dim:=2 potential:=quartic particle_mass:=437.722 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0045 f_k:=28 f_kt:=14 $Z2
sph real_quart2d_tang_4 total_robots:=4 dim:=2 potential:=quartic particle_mass:=435.313 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0142 f_k:=5 f_kt:=6 $Z2
sph real_quart2d_tang_5 total_robots:=5 dim:=2 potential:=quartic particle_mass:=437.687 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.035 f_k:=2 f_kt:=3 $Z2
# Circulação: espaço
sph real_quart3d_tang_2 total_robots:=2 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0009 f_k:=320 f_kt:=16 $Z3
sph real_quart3d_tang_3 total_robots:=3 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0045 f_k:=28 f_kt:=5 $Z3
sph real_quart3d_tang_4 total_robots:=4 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.0142 f_k:=5 f_kt:=2 $Z3
sph real_quart3d_tang_5 total_robots:=5 dim:=3 potential:=quartic particle_mass:=330 smoothing_h:=0.5 gamma:=3 zetta_extra:=60 xc:=0.6 yc:=0.6 zc:=1 R:=0.035 f_k:=5 f_kt:=1 $Z3
```

## 6. Gráficos e indicadores

No host:

```bash
python3 src/plot_plain_sph.py src/logs/runs/<run_id> --no-open     # sem <run_id>: última rodada
```

As figuras e o `indicadores_<label>.csv` (d_min, max e_i, ρ̄/ρ₀, v̄, t_c, d_obs…) vão para
`src/logs/runs/<run_id>/plots/`. Opções: `--collide` (limiar de colisão, padrão 0,10 m), `--tc-eps` (limiar
de t_c, padrão 0,10 m), `--final-window` (janela final, padrão 10 s), `--pdf` (figuras vetoriais).

## 7. Testes antigos com `dev_tmux.sh`

O `dev_tmux.sh` sobe tudo em uma sessão tmux (simulação, Crazyswarm2, controlador e decolagem automática).
Rode do host, em `~/SPH_Project`. Cada parâmetro é uma variável de ambiente (padrões em `swarm.conf`).

```bash
./dev_tmux.sh up              # CrazySim + Crazyswarm2 + um nó de controle por drone
./dev_tmux.sh up-particles    # dinâmica simplificada (sem física)
./dev_tmux.sh show            # mostra a configuração resolvida, sem iniciar nada
./dev_tmux.sh analyze         # números da última rodada
./dev_tmux.sh stop            # encerra tudo (ou Ctrl+b K dentro do tmux)
./dev_tmux.sh attach          # volta a uma sessão aberta (Ctrl+b d sai sem encerrar)
./dev_tmux.sh test [nome]     # testes de movimento no CrazySim, sem o SPH
```

Principais variáveis: `N_DRONES`, `SPH_DIM` (2/3), `SPH_H`, `SPH_SPACING`, `SPH_MASS` (número ou `auto`),
`SPH_GAMMA`, `SPH_ZETTA`, `SPH_GAIN`, `POTENTIAL` (`default`/`quartic`), `POT_XC/YC/ZC`, `POT_R`, `POT_K`,
`POT_KT`, `OBSTACLES` (`"x,y/x,y,z"`), `OBS_SIGMA`, `SIM_CLOCK` (`wall`/`gz`), `GZ_GUI`, `RVIZ`, `RUN_LABEL`.
3D: `MAX_VZ`, `Z_MIN`, `Z_MAX`, `Z_MARGIN`, `Z_WALL_K`.

### 7.1 Ensaios da monografia

```bash
export SPH_GAIN=0.5

# ---------- dinâmica simplificada ----------
N_DRONES=20 SPH_H=0.3 SPH_MASS=131.946 SPH_SPACING=0.3 SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=10.0 POT_R=0.96 RUN_LABEL=p_circ_20 ./dev_tmux.sh up-particles
N_DRONES=8 SPH_DIM=3 SPH_H=0.3 SPH_MASS=83.5371 SPH_SPACING=0.5 SPH_GAMMA=7.0 SPH_ZETTA=40.0 POT_K=2.0 POT_R=0.4544 POT_ZC=1.2 RUN_LABEL=p_esfera ./dev_tmux.sh up-particles
N_DRONES=12 POTENTIAL=quartic SPH_H=0.3 SPH_MASS=172.382 SPH_SPACING=0.4 SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=10.0 POT_R=0.23 RUN_LABEL=p_quart2d ./dev_tmux.sh up-particles
N_DRONES=8 SPH_DIM=3 POTENTIAL=quartic SPH_H=0.5 SPH_MASS=330 SPH_SPACING=0.5 SPH_GAMMA=3.0 SPH_ZETTA=40.0 POT_K=10.0 POT_R=0.23 POT_XC=1.5 POT_YC=1.5 RUN_LABEL=p_quart3d ./dev_tmux.sh up-particles
N_DRONES=12 POTENTIAL=quartic SPH_H=0.3 SPH_MASS=172.382 SPH_SPACING=0.4 SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=7.0 POT_R=0.32 POT_KT=2.0 RUN_LABEL=p_quart2d_tang ./dev_tmux.sh up-particles
N_DRONES=8 SPH_DIM=3 POTENTIAL=quartic SPH_H=0.5 SPH_MASS=330 SPH_SPACING=0.5 SPH_GAMMA=3.0 SPH_ZETTA=40.0 POT_K=10.0 POT_R=0.23 POT_XC=1.5 POT_YC=1.5 POT_KT=1.0 RUN_LABEL=p_quart3d_tang ./dev_tmux.sh up-particles
N_DRONES=14 SPH_H=0.3 SPH_MASS=194.322 SPH_SPACING=0.5 SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=10.0 POT_R=1.123 OBSTACLES="0.65,1.35/1.35,0.65" OBS_SIGMA=2.0 RUN_LABEL=p_obs_circ_portao_14 ./dev_tmux.sh up-particles
N_DRONES=15 POTENTIAL=quartic SPH_H=0.3 SPH_MASS=194.322 SPH_SPACING=0.5 SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=3.0 POT_KT=7.0 POT_R=6.866 POT_XC=3.8 POT_YC=3.8 OBSTACLES="3.8,5.14" OBS_SIGMA=2.0 RUN_LABEL=p_obs_quart2d_tang_15 ./dev_tmux.sh up-particles

# ---------- CrazySim (rodadas originais, 3 a 8 veículos) ----------
N_DRONES=8 SPH_H=0.3 SPH_MASS=131.792 SPH_SPACING=0.3 SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=10.0 POT_R=0.39 RUN_LABEL=cs_circ_8 ./dev_tmux.sh up
N_DRONES=8 SPH_DIM=3 SPH_H=0.3 SPH_MASS=80.1952 SPH_SPACING=0.5 SPH_GAMMA=7.0 SPH_ZETTA=40.0 POT_K=2.0 POT_R=0.41 POT_ZC=1.2 RUN_LABEL=cs_esfera_8 ./dev_tmux.sh up
N_DRONES=6 POTENTIAL=quartic SPH_H=0.5 SPH_MASS=437.722 SPH_SPACING=0.6 SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=20.0 POT_R=0.072 RUN_LABEL=cs_quart2d_6 ./dev_tmux.sh up
N_DRONES=8 SPH_DIM=3 POTENTIAL=quartic SPH_H=0.5 SPH_MASS=330 SPH_SPACING=0.5 SPH_GAMMA=3.0 SPH_ZETTA=40.0 SPH_GAIN=1.0 POT_K=10.0 POT_R=0.23 POT_XC=1.5 POT_YC=1.5 RUN_LABEL=cs_quart3d_8 ./dev_tmux.sh up
N_DRONES=6 POTENTIAL=quartic SPH_H=0.5 SPH_MASS=437.722 SPH_SPACING=0.6 SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=10.0 POT_R=0.072 POT_KT=2.0 RUN_LABEL=cs_quart2d_tang_6 ./dev_tmux.sh up
N_DRONES=8 SPH_DIM=3 POTENTIAL=quartic SPH_H=0.5 SPH_MASS=330 SPH_SPACING=0.5 SPH_GAMMA=3.0 SPH_ZETTA=40.0 POT_K=10.0 POT_R=0.23 POT_XC=1.5 POT_YC=1.5 POT_KT=1.0 RUN_LABEL=cs_quart3d_tang_8 ./dev_tmux.sh up
N_DRONES=3 SPH_H=0.3 SPH_MASS=194.322 SPH_SPACING=0.5 SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=10.0 POT_R=0.29 OBSTACLES="0.65,1.35/1.35,0.65" OBS_SIGMA=2.0 RUN_LABEL=cs_obs_circ_portao_3 ./dev_tmux.sh up
N_DRONES=3 POTENTIAL=quartic SPH_H=0.3 SPH_MASS=194.322 SPH_SPACING=0.5 SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=15.0 POT_KT=7.0 POT_R=0.4 OBSTACLES="2.0,2.76" OBS_SIGMA=2.0 RUN_LABEL=cs_obs_quart2d_tang_3 ./dev_tmux.sh up
```

### 7.2 Testes de 3 a 5 drones (arena)

```bash
# ---------- CrazySim ----------
N_DRONES=3 SPH_MASS=auto SPH_GAIN=0.5 POT_K=10.0 POT_R=0.346 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=cs_circ_3 ./dev_tmux.sh up
N_DRONES=4 SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=60.0 POT_R=0.0142 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=cs_quart2d_4 ./dev_tmux.sh up
N_DRONES=4 SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=60.0 POT_KT=5.0 POT_R=0.0142 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=cs_quart2d_tang_4 ./dev_tmux.sh up
N_DRONES=4 SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.0142 POT_XC=0.7 POT_YC=0.7 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=cs_quart3d_4 ./dev_tmux.sh up
N_DRONES=4 SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.0142 POT_XC=0.7 POT_YC=0.7 POT_K=60.0 POT_KT=2.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=cs_quart3d_tang_4 ./dev_tmux.sh up
N_DRONES=5 SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=20.0 POT_R=0.035 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=cs_quart2d_5 ./dev_tmux.sh up
N_DRONES=5 SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.035 POT_XC=0.7 POT_YC=0.7 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=cs_quart3d_5 ./dev_tmux.sh up
N_DRONES=5 SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=20.0 POT_R=0.035 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 POT_KT=3.0 RUN_LABEL=cs_quart2d_tang_5 ./dev_tmux.sh up
N_DRONES=5 SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.035 POT_XC=0.65 POT_YC=0.65 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 POT_KT=1.0 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=cs_quart3d_tang_5 ./dev_tmux.sh up

# ---------- obstáculos ----------
export SPH_H=0.3 SPH_SPACING=0.5 SPH_GAIN=0.5 OBS_SIGMA=2.0
N_DRONES=4 SPH_MASS=auto SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=10.0 POT_R=0.354 OBSTACLES="0.65,1.35/1.35,0.65" SIM_CLOCK=gz RUN_LABEL=cs_obs_circ_portao_4 ./dev_tmux.sh up
N_DRONES=5 SPH_MASS=auto SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=10.0 POT_R=0.425 OBSTACLES="0.65,1.35/1.35,0.65" SIM_CLOCK=gz RUN_LABEL=cs_obs_circ_portao_5 ./dev_tmux.sh up
N_DRONES=4 POTENTIAL=quartic SPH_MASS=auto SPH_GAMMA=3.0 SPH_ZETTA=50.0 POT_K=15.0 POT_KT=7.0 POT_R=0.4 OBSTACLES="2.0,2.76" RUN_LABEL=cs_obs_quart2d_tang_4 ./dev_tmux.sh up
N_DRONES=6 SPH_DIM=3 SPH_MASS=auto SPH_GAMMA=7.0 SPH_ZETTA=40.0 POT_K=2.0 POT_R=0.36 POT_ZC=1.2 OBSTACLES="2.36,1.64,1.2" RUN_LABEL=p_obs_esfera_6 ./dev_tmux.sh up-particles
N_DRONES=3 SPH_DIM=3 SPH_MASS=auto SPH_GAMMA=7.0 SPH_ZETTA=40.0 POT_K=2.0 POT_R=0.29 POT_ZC=1.2 OBSTACLES="1.0,1.0,0.85" RUN_LABEL=cs_obs_esfera_3 ./dev_tmux.sh up
N_DRONES=5 SPH_DIM=3 POTENTIAL=quartic SPH_MASS=70 SPH_H=0.3 SPH_GAMMA=3.0 SPH_ZETTA=40.0 POT_K=3.7 POT_R=0.215 POT_XC=1.5 POT_YC=1.5 OBSTACLES="1.5,0.87,1.26" RUN_LABEL=cs_obs_quart3d_5 ./dev_tmux.sh up
N_DRONES=14 SPH_DIM=3 POTENTIAL=quartic SPH_MASS=70 SPH_H=0.3 SPH_GAMMA=3.0 SPH_ZETTA=40.0 POT_K=3.7 POT_R=0.215 POT_XC=1.5 POT_YC=1.5 OBSTACLES="1.5,0.87,1.26" RUN_LABEL=p_obs_quart3d_14 ./dev_tmux.sh up-particles
```

### 7.3 Drones reais com `real_tmux.sh`

```bash
./real_tmux.sh up                                 # Crazyswarm2 com config/crazyflies_real.yaml
./real_tmux.sh test frente_tras --v 0.2 --tempo 2 # testes de movimento (em outro terminal ou Ctrl+b d)
./real_tmux.sh takeoff                            # depois, a linha do teste SPH abaixo
./real_tmux.sh land
./real_tmux.sh stop

SPH_MASS=auto SPH_GAIN=0.5 POT_K=10.0 POT_R=0.346 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=real_circ_3 ./real_tmux.sh sph
SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=60.0 POT_R=0.0142 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=real_quart2d_4 ./real_tmux.sh sph
SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.0142 POT_XC=0.7 POT_YC=0.7 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=real_quart3d_4 ./real_tmux.sh sph
SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=20.0 POT_R=0.035 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 RUN_LABEL=real_quart2d_5 ./real_tmux.sh sph
SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.035 POT_XC=0.7 POT_YC=0.7 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=real_quart3d_5 ./real_tmux.sh sph
SPH_MASS=auto SPH_GAIN=0.5 POTENTIAL=quartic POT_K=20.0 POT_R=0.035 SPH_GAMMA=3.0 SPH_SPACING=0.6 SPH_H=0.5 SPH_ZETTA=50.0 POT_KT=3.0 RUN_LABEL=real_quart2d_tang_5 ./real_tmux.sh sph
SPH_GAIN=0.5 SPH_GAMMA=3.0 SPH_DIM=3 POTENTIAL=quartic POT_R=0.035 POT_XC=0.65 POT_YC=0.65 POT_K=60.0 SPH_MASS=330 SPH_H=0.5 SPH_SPACING=0.5 POT_KT=1.0 Z_MIN=0.2 Z_MARGIN=0.1 Z_MAX=1.5 Z_WALL_K=8 RUN_LABEL=real_quart3d_tang_5 ./real_tmux.sh sph
```