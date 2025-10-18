import pandas as pd
import glob

frames = [pd.read_csv(f) for f in glob.glob("logs/*.csv")]
data = pd.concat(frames)

save_path = "line_test_full_data.csv"
data.to_csv(save_path, index=False)


import pandas as pd
data = pd.read_csv("line_test.csv")
print(data.head())
t = data["t"].to_numpy()
err = data["error"].to_numpy()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Bayesian Optimization (Gaussian Process + EI) dla strojenia PIDController.
Wczytuje CSV z logami (t,error) lub (t,L,R), liczy koszt i dobiera (kp,ki,kd,d_alpha).

Autor: Ty + ChatGPT :)
"""

import argparse
import glob
import os
import sys
from typing import List, Tuple, Dict

import numpy as np
import pandas as pd

# --- zależności BO ---
try:
    from skopt import gp_minimize
    from skopt.space import Real
    from skopt.utils import use_named_args
except Exception as e:
    print("\n[ERROR] Brakuje pakietu 'scikit-optimize'. Zainstaluj:")
    print("  pip install scikit-optimize\n")
    raise

# --- import Twojej klasy PIDController ---
def ensure_pid_on_path(pid_path: str):
    if pid_path:
        abs_path = os.path.abspath(pid_path)
        if abs_path not in sys.path:
            sys.path.append(abs_path)

def import_pid_controller():
    try:
        from pid_controller import PIDController  # noqa: F401
        return PIDController
    except Exception as e:
        print("\n[ERROR] Nie mogę zaimportować PIDController z 'pid_controller.py'.")
        print("       Użyj --pid-path, aby podać katalog z tym plikiem.")
        print("       Przykład: --pid-path ../controllers\n")
        raise

# --- pomocnicze: wczytanie i sanityzacja danych ---
def load_logs(paths: List[str]) -> List[pd.DataFrame]:
    files = []
    for p in paths:
        files.extend(glob.glob(p))
    if not files:
        raise FileNotFoundError("Nie znaleziono żadnych plików logów dla podanych wzorców/ścieżek.")

    datasets = []
    for f in sorted(files):
        df = pd.read_csv(f)
        if "t" not in df.columns:
            raise ValueError(f"{f}: brak kolumny 't'.")
        has_error = "error" in df.columns
        if not has_error and not {"L", "R"}.issubset(df.columns):
            raise ValueError(f"{f}: potrzebuję kolumn 'error' albo obu 'L' i 'R'.")

        # Wylicz error jeśli nie istnieje
        if not has_error:
            df["error"] = df["R"] - df["L"]

        # Usuń NaN / inf
        df = df.replace([np.inf, -np.inf], np.nan).dropna(subset=["t", "error"])
        if len(df) < 5:
            print(f"[WARN] {f}: bardzo mało próbek po czyszczeniu ({len(df)}). Pomijam.")
            continue

        # Uporządkuj czas i upewnij się, że rośnie
        df = df.sort_values("t").reset_index(drop=True)
        datasets.append(df)

    if not datasets:
        raise RuntimeError("Po wczytaniu i czyszczeniu nie zostały żadne dane.")
    return datasets

# --- koszt: ważone MSE błędu + kary za szarpanie i saturację sterowania ---
def make_cost_fn(PIDController, umax: float, we: float, wu: float, wdu: float, wsat: float, derivative_on_measurement: bool):
    def cost_for_params_single(df: pd.DataFrame, kp: float, ki: float, kd: float, d_alpha: float, integral_limit: float) -> float:
        t = df["t"].to_numpy()
        err = df["error"].to_numpy()
        # dt z mediany (załóż stały krok)
        if len(t) < 2:
            return np.inf
        dt = float(np.median(np.diff(t)))
        if dt <= 0:
            return np.inf

        pid = PIDController(kp, ki, kd, dt,
                            integral_limit=integral_limit,
                            d_alpha=d_alpha,
                            derivative_on_measurement=derivative_on_measurement)
        pid.reset()

        J = 0.0
        u_prev = 0.0
        n = 0
        for e in err:
            # setpoint=0 -> chcemy error ~ 0
            u = pid.compute(0.0, e)
            # saturacja (jak w realnej pętli)
            if u > umax:
                u = umax
                J += wsat
            elif u < -umax:
                u = -umax
                J += wsat

            # koszt: błąd + "siła" sterowania + gładkość sterowania
            J += we*(e*e) + wu*(u*u) + wdu*((u - u_prev)*(u - u_prev))
            u_prev = u
            n += 1

        return J / max(n, 1)

    def cost_for_params_all(datasets: List[pd.DataFrame], kp: float, ki: float, kd: float, d_alpha: float, integral_limit: float) -> float:
        vals = [cost_for_params_single(df, kp, ki, kd, d_alpha, integral_limit) for df in datasets]
        return float(np.mean(vals))
    return cost_for_params_all

def main():
    ap = argparse.ArgumentParser(description="Bayesian Optimization tuner dla PIDController (EV3 line-follow).")
    ap.add_argument("--logs", nargs="+", required=True,
                    help="Ścieżki/wzorce do CSV z logami (np. logs/*.csv). Wymaga kolumn: t i (error lub L,R).")
    ap.add_argument("--pid-path", default="",
                    help="Katalog, w którym jest pid_controller.py (jeśli nie w tym samym co tuner).")
    ap.add_argument("--calls", type=int, default=40, help="Łączna liczba wywołań funkcji celu (prób).")
    ap.add_argument("--init-points", type=int, default=10, help="Liczba losowych punktów startowych.")
    ap.add_argument("--umax", type=float, default=20.0, help="Maksymalna wartość sterowania (np. kąt skrętu w stopniach).")
    ap.add_argument("--we", type=float, default=1.0, help="Waga dla błędu e^2.")
    ap.add_argument("--wu", type=float, default=0.01, help="Waga dla sterowania u^2.")
    ap.add_argument("--wdu", type=float, default=0.1, help="Waga dla różnicy sterowań (u - u_prev)^2.")
    ap.add_argument("--wsat", type=float, default=5.0, help="Kara za saturację (gdy |u| >= umax).")
    ap.add_argument("--derivative-on-measurement", action="store_true",
                    help="Użyj pochodnej na pomiarze (często stabilniej dla zaszumionych czujników).")
    # zakresy poszukiwań:
    ap.add_argument("--kp", nargs=2, type=float, default=[0.2, 3.0], help="Zakres Kp (min max).")
    ap.add_argument("--ki", nargs=2, type=float, default=[0.0, 0.5], help="Zakres Ki (min max).")
    ap.add_argument("--kd", nargs=2, type=float, default=[0.0, 1.0], help="Zakres Kd (min max).")
    ap.add_argument("--d-alpha", nargs=2, type=float, default=[0.0, 0.9], help="Zakres d_alpha (min max).")
    ap.add_argument("--integral-limit", nargs=2, type=float, default=[5.0, 200.0], help="Zakres integral_limit (min max).")
    ap.add_argument("--random-state", type=int, default=42, help="Seed RNG dla powtarzalności.")
    args = ap.parse_args()

    # przygotuj import PID
    ensure_pid_on_path(args.pid_path)
    PIDController = import_pid_controller()

    # wczytaj dane (lista DataFrame)
    datasets = load_logs(args.logs)

    # koszt dla wszystkich zbiorów (średnia)
    cost_fn = make_cost_fn(
        PIDController=PIDController,
        umax=args.umax,
        we=args.we, wu=args.wu, wdu=args.wdu, wsat=args.wsat,
        derivative_on_measurement=args.derivative_on_measurement
    )

    # przestrzeń poszukiwań
    space = [
        Real(args.kp[0], args.kp[1], name="kp"),
        Real(args.ki[0], args.ki[1], name="ki"),
        Real(args.kd[0], args.kd[1], name="kd"),
        Real(args.d_alpha[0], args.d_alpha[1], name="d_alpha"),
        Real(args.integral_limit[0], args.integral_limit[1], name="integral_limit"),
    ]

    @use_named_args(space)
    def objective(**p):
        val = cost_fn(datasets, **p)
        return float(val)

    print("\n=== Start Bayesian Optimization ===")
    print(f"Logi: {len(datasets)} plik(ów)")
    print(f"Próby: {args.calls}, punkty startowe: {args.init_points}")
    print(f"Zakresy: Kp{tuple(args.kp)}, Ki{tuple(args.ki)}, Kd{tuple(args.kd)}, d_alpha{tuple(args.d_alpha)}, integral_limit{tuple(args.integral_limit)}")
    print(f"Umax={args.umax}, w_e={args.we}, w_u={args.wu}, w_du={args.wdu}, w_sat={args.wsat}")
    print(f"Derivative on measurement: {bool(args.derivative_on_measurement)}\n")

    res = gp_minimize(
        objective,
        dimensions=space,
        n_calls=args.calls,
        n_initial_points=args.init_points,
        acq_func="EI",
        noise=1e-6,
        random_state=args.random_state
    )

    best = {
        "kp": res.x[0],
        "ki": res.x[1],
        "kd": res.x[2],
        "d_alpha": res.x[3],
        "integral_limit": res.x[4],
    }

    print("\n=== Wynik ===")
    print("Najlepsze parametry (średnio na wszystkich logach):")
    print("  kp = {:.6f}\n  ki = {:.6f}\n  kd = {:.6f}\n  d_alpha = {:.6f}\n  integral_limit = {:.6f}".format(
        best["kp"], best["ki"], best["kd"], best["d_alpha"], best["integral_limit"]
    ))
    print("Szacowany koszt (niżej = lepiej): {:.6f}".format(res.fun))

    # podpowiedź do użycia w Twoim skrypcie EV3
    print("\nWklej do swojego programu na EV3 (przykład):")
    print("pid = PIDController(kp={:.6f}, ki={:.6f}, kd={:.6f}, dt=DT, d_alpha={:.6f}, integral_limit={:.6f}, derivative_on_measurement={})".format(
        best["kp"], best["ki"], best["kd"], best["d_alpha"], best["integral_limit"], bool(args.derivative_on_measurement)
    ))
    print("\nUwaga: podmień DT na rzeczywisty krok pętli sterującej (sekundy).")

if __name__ == "__main__":
    main()
