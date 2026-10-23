#!/usr/bin/env python3
"""Frente, tras, um lado, o outro lado, com o Commander de velocidade.  Termina onde comecou:
  frente      +x por --tempo s
  tras        -x por --tempo s      (de volta ao centro)
  lado        +y por --tempo s
  outro lado  -y por 2 x --tempo s  (passa pelo centro e vai ao outro lado)
  volta       +y por --tempo s      (de volta ao centro)

    python3 frente_tras_lados.py --v 0.2 --tempo 3 --pausa 2
"""
from testkit import Teste


def args(p):
    p.add_argument('--v', type=float, default=0.2, help='m/s')
    p.add_argument('--tempo', type=float, default=3.0, help='s de cada movimento (desloca v*tempo m)')
    p.add_argument('--pausa', type=float, default=2.0, help='s parado entre os movimentos')
    p.add_argument('--repeticoes', type=int, default=1)


with Teste.from_args(__doc__, args) as t:
    a = t.args
    movimentos = [('frente', (+a.v, 0.0), a.tempo), ('tras', (-a.v, 0.0), a.tempo),
                  ('lado', (0.0, +a.v), a.tempo), ('outro lado', (0.0, -a.v), 2 * a.tempo),
                  ('volta ao centro', (0.0, +a.v), a.tempo)]
    t.decolar()
    t.parar(a.pausa)
    for k in range(a.repeticoes):
        for nome, v, dur in movimentos:
            t.velocidade(v, dur, fase=f'{nome} {k + 1}')
            t.parar(a.pausa)
    t.pousar()