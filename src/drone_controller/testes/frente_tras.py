#!/usr/bin/env python3
"""Frente e tras com o Commander de velocidade: +v em x por --tempo s, para, -v em x por --tempo s.

    python3 frente_tras.py --v 0.2 --tempo 3 --pausa 2 --repeticoes 1
"""
from testkit import Teste


def args(p):
    p.add_argument('--v', type=float, default=0.2, help='m/s')
    p.add_argument('--tempo', type=float, default=3.0, help='s em cada sentido (desloca v*tempo m)')
    p.add_argument('--pausa', type=float, default=2.0, help='s parado entre os movimentos')
    p.add_argument('--repeticoes', type=int, default=1)


with Teste.from_args(__doc__, args) as t:
    a = t.args
    t.decolar()
    t.parar(a.pausa)
    for k in range(a.repeticoes):
        t.velocidade((+a.v, 0.0), a.tempo, fase=f'frente {k + 1}')
        t.parar(a.pausa)
        t.velocidade((-a.v, 0.0), a.tempo, fase=f'tras {k + 1}')
        t.parar(a.pausa)
    t.pousar()