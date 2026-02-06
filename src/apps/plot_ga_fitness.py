import csv
from pathlib import Path

import matplotlib.pyplot as plt


def main():
    candidates = [
        Path("ga_fitness_log.csv"),
        Path("..").resolve() / "ga_fitness_log.csv",
        Path("..").resolve().parent / "ga_fitness_log.csv",
    ]
    log_path = next((p for p in candidates if p.exists()), None)
    if log_path is None:
        raise SystemExit("ga_fitness_log.csv not found. Check build/bin or project root.")

    generations = []
    best_fitness = []

    with log_path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            generations.append(int(row["generation"]))
            best_fitness.append(float(row["best_fitness"]))

    plt.figure(figsize=(8, 4.5))
    plt.plot(generations, best_fitness, linewidth=2)
    plt.title("Best Fitness per Generation")
    plt.xlabel("Generation")
    plt.ylabel("Best Fitness")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.savefig("ga_fitness_plot.png", dpi=150)
    plt.show()


if __name__ == "__main__":
    main()
