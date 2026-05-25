#!/usr/bin/env python3
"""Generate plots and a report from step loss sweep CSV results."""

from __future__ import annotations

import argparse
import datetime as dt
import math
import os
import tempfile
import warnings
from pathlib import Path


pd = None
np = None
plt = None
LogNorm = None
ListedColormap = None
BoundaryNorm = None


REQUIRED_COLUMNS = ("speed_mm_s", "accel_mm_s2", "current_ma")
FIGURE_NAMES = {
    "overview": "01_overview_scatter",
    "error_heatmap": "02_error_heatmap_by_current",
    "result_heatmap": "03_result_heatmap_by_current",
    "max_speed_current": "04_max_stable_speed_by_current",
    "max_speed_accel": "05_max_stable_speed_by_accel",
    "error_speed": "06_error_vs_speed_by_current",
}


def main() -> int:
    args = parse_args()
    ensure_dependencies()

    out_dir = resolve_output_dir(args.out)
    figures_dir = out_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)

    raw_df = load_results(args.csv)
    raw_df = filter_current_list(raw_df, args.current_list)
    classified_df = classify_by_error(raw_df, args.ok_error_threshold, args.ng_error_threshold)
    df = aggregate_duplicate_conditions(classified_df)
    df = calculate_margin_score(df)

    raw_out = out_dir / "raw_results.csv"
    classified_df.to_csv(raw_out, index=False)

    figure_paths = {
        "overview": figures_dir / f"{FIGURE_NAMES['overview']}.{args.format}",
        "error_heatmap": figures_dir / f"{FIGURE_NAMES['error_heatmap']}.{args.format}",
        "result_heatmap": figures_dir / f"{FIGURE_NAMES['result_heatmap']}.{args.format}",
        "max_speed_current": figures_dir / f"{FIGURE_NAMES['max_speed_current']}.{args.format}",
        "max_speed_accel": figures_dir / f"{FIGURE_NAMES['max_speed_accel']}.{args.format}",
        "error_speed": figures_dir / f"{FIGURE_NAMES['error_speed']}.{args.format}",
    }

    plot_overview_scatter(df, str(figure_paths["overview"]), args.title)
    error_heatmap_generated = plot_error_heatmap_by_current(
        df,
        str(figure_paths["error_heatmap"]),
        args.show_cell_labels,
        args.error_scale,
        args.title,
    )
    plot_result_heatmap_by_current(df, str(figure_paths["result_heatmap"]), args.show_cell_labels, args.title)
    max_by_current = plot_max_speed_by_current(df, str(figure_paths["max_speed_current"]), args.title)
    max_by_accel = plot_max_speed_by_accel(df, str(figure_paths["max_speed_accel"]), args.title)
    plot_error_vs_speed_by_current(df, str(figure_paths["error_speed"]), args.title)

    recommended_df = select_recommended_settings(df, args.min_margin_score)
    recommended_df.to_csv(out_dir / "safe_region_table.csv", index=False)

    write_markdown_report(
        df=df,
        recommended_df=recommended_df,
        out_path=str(out_dir / "report.md"),
        figures_dir="figures",
        thresholds=(args.ok_error_threshold, args.ng_error_threshold),
        input_csv=args.csv,
        title=args.title,
        error_heatmap_generated=error_heatmap_generated,
        figure_ext=args.format,
        max_by_current=max_by_current,
        max_by_accel=max_by_accel,
    )

    print(f"Report: {out_dir / 'report.md'}")
    print(f"Raw results: {raw_out}")
    print(f"Recommended settings: {out_dir / 'safe_region_table.csv'}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot step loss sweep CSV results.")
    parser.add_argument("--csv", required=True, help="input result CSV")
    parser.add_argument("--out", default="", help="output folder or reports parent folder")
    parser.add_argument("--title", default="", help="optional plot title prefix")
    parser.add_argument("--show-cell-labels", action="store_true", help="show values inside heatmap cells")
    parser.add_argument("--current-list", default="", help="comma-separated current_ma values to plot")
    parser.add_argument("--ok-error-threshold", type=float, default=0.2, help="maximum abs error for OK")
    parser.add_argument("--ng-error-threshold", type=float, default=1.0, help="abs error above this is NG")
    parser.add_argument("--min-margin-score", type=float, default=0.5, help="minimum margin score for recommendations")
    parser.add_argument("--error-scale", choices=("linear", "log"), default="linear", help="error heatmap color scale")
    parser.add_argument("--format", choices=("png", "svg", "pdf"), default="png", help="figure output format")
    return parser.parse_args()


def ensure_dependencies() -> None:
    global pd, np, plt, LogNorm, ListedColormap, BoundaryNorm
    missing: list[str] = []
    try:
        import pandas as pandas_module  # type: ignore
    except ImportError:
        missing.append("pandas")
    try:
        import numpy as numpy_module  # type: ignore
    except ImportError:
        missing.append("numpy")
    try:
        cache_dir = tempfile.mkdtemp(prefix="plot_step_loss_mpl_")
        os.environ.setdefault("MPLCONFIGDIR", cache_dir)
        os.environ.setdefault("XDG_CACHE_HOME", cache_dir)
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as pyplot_module  # type: ignore
        from matplotlib.colors import BoundaryNorm as BoundaryNormClass  # type: ignore
        from matplotlib.colors import ListedColormap as ListedColormapClass  # type: ignore
        from matplotlib.colors import LogNorm as LogNormClass  # type: ignore
    except ImportError:
        missing.append("matplotlib")

    if missing:
        unique = ", ".join(sorted(set(missing)))
        raise SystemExit(
            f"Missing Python packages: {unique}\n"
            "Install plotting dependencies with:\n"
            "  python3 -m pip install -r requirements-plot.txt\n"
        )

    pd = pandas_module
    np = numpy_module
    plt = pyplot_module
    LogNorm = LogNormClass
    ListedColormap = ListedColormapClass
    BoundaryNorm = BoundaryNormClass


def resolve_output_dir(out_arg: str) -> Path:
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    if not out_arg:
        return Path("reports") / f"step_loss_sweep_{timestamp}"

    out = Path(out_arg)
    if out.name == "reports":
        return out / f"step_loss_sweep_{timestamp}"
    if out.name.startswith("step_loss_sweep_"):
        return out
    return out


def load_results(csv_path: str) -> pd.DataFrame:
    """CSVを読み込み、必要な列を検証し、resultとerror列を正規化する。"""
    df = pd.read_csv(csv_path)
    missing = [col for col in REQUIRED_COLUMNS if col not in df.columns]
    if missing:
        raise SystemExit(f"Input CSV is missing required columns: {', '.join(missing)}")

    for col in REQUIRED_COLUMNS:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    if df[list(REQUIRED_COLUMNS)].isna().any().any():
        raise SystemExit("speed_mm_s, accel_mm_s2, and current_ma must be numeric")

    if "result" not in df.columns and "final_result" in df.columns:
        df["result"] = df["final_result"]

    if "result" in df.columns:
        df["source_result_code"] = df["result"].apply(normalize_result)
    else:
        df["source_result_code"] = np.nan

    return ensure_error_columns(df)


def normalize_result(value) -> float:
    """OKなら1、WARNなら0.5、NGなら0、不明ならNaNを返す。"""
    if pd.isna(value):
        return np.nan
    text = str(value).strip()
    lowered = text.lower()
    if lowered in {"ok", "pass", "success", "1", "true"}:
        return 1.0
    if lowered in {"warn", "warning"}:
        return 0.5
    if lowered in {"ng", "fail", "stall", "step_loss", "0", "false"}:
        return 0.0
    return np.nan


def ensure_error_columns(df: pd.DataFrame) -> pd.DataFrame:
    """error_mm / abs_error_mm を整備する。abs_error_mm がなければ error_mm から作る。"""
    df = df.copy()
    if "abs_error_mm" in df.columns:
        df["abs_error_mm"] = pd.to_numeric(df["abs_error_mm"], errors="coerce").abs()
    elif "error_mm" in df.columns:
        df["error_mm"] = pd.to_numeric(df["error_mm"], errors="coerce")
        df["abs_error_mm"] = df["error_mm"].abs()
    elif "final_error_mm" in df.columns:
        df["final_error_mm"] = pd.to_numeric(df["final_error_mm"], errors="coerce")
        df["abs_error_mm"] = df["final_error_mm"].abs()
        if "error_mm" not in df.columns:
            df["error_mm"] = df["final_error_mm"]
    else:
        df["abs_error_mm"] = np.nan

    if "error_mm" in df.columns:
        df["error_mm"] = pd.to_numeric(df["error_mm"], errors="coerce")
    else:
        df["error_mm"] = np.nan
    return df


def classify_by_error(
    df: pd.DataFrame,
    ok_error_threshold: float,
    ng_error_threshold: float,
) -> pd.DataFrame:
    """ずれ量とresult列から final_result を OK/WARN/NG に分類する。"""
    df = df.copy()
    result_code = df["source_result_code"].copy()
    has_error = df["abs_error_mm"].notna()

    error_code = pd.Series(np.nan, index=df.index, dtype=float)
    error_code.loc[has_error & (df["abs_error_mm"] <= ok_error_threshold)] = 1.0
    error_code.loc[has_error & (df["abs_error_mm"] > ok_error_threshold) & (df["abs_error_mm"] <= ng_error_threshold)] = 0.5
    error_code.loc[has_error & (df["abs_error_mm"] > ng_error_threshold)] = 0.0

    combined = result_code.copy()
    combined.loc[combined.isna()] = error_code.loc[combined.isna()]
    explicit_ng = result_code == 0.0
    combined.loc[explicit_ng] = 0.0
    with_error = error_code.notna()
    combined.loc[with_error & ~explicit_ng] = np.minimum(combined.loc[with_error & ~explicit_ng].fillna(1.0), error_code.loc[with_error & ~explicit_ng])

    df["result_code"] = combined
    df["final_result"] = df["result_code"].apply(result_label)
    df["is_ok"] = df["result_code"] == 1.0
    return df


def result_label(code: float) -> str:
    if pd.isna(code):
        return "UNKNOWN"
    if code == 1.0:
        return "OK"
    if code == 0.5:
        return "WARN"
    if code == 0.0:
        return "NG"
    return "UNKNOWN"


def aggregate_duplicate_conditions(df: pd.DataFrame) -> pd.DataFrame:
    """同一条件の重複測定を安全側に集約する。resultは最悪値、abs_error_mmは最大値を使う。"""
    grouped = []
    for key, group in df.groupby(list(REQUIRED_COLUMNS), dropna=False):
        result_codes = group["result_code"].dropna()
        if (result_codes == 0).any():
            result_code = 0.0
        elif (result_codes == 0.5).any():
            result_code = 0.5
        elif len(result_codes) > 0 and (result_codes == 1).all():
            result_code = 1.0
        else:
            result_code = np.nan

        abs_error = group["abs_error_mm"].max() if "abs_error_mm" in group else np.nan
        error_mm = np.nan
        if "error_mm" in group and group["error_mm"].notna().any():
            idx = group["error_mm"].abs().idxmax()
            error_mm = group.loc[idx, "error_mm"]

        row = {
            "speed_mm_s": key[0],
            "accel_mm_s2": key[1],
            "current_ma": key[2],
            "result_code": result_code,
            "final_result": result_label(result_code),
            "is_ok": result_code == 1.0,
            "abs_error_mm": abs_error,
            "error_mm": error_mm,
            "sample_count": len(group),
        }
        for col in optional_columns(group):
            row[col] = join_unique(group[col])
        grouped.append(row)
    return pd.DataFrame(grouped).sort_values(list(REQUIRED_COLUMNS)).reset_index(drop=True)


def optional_columns(df: pd.DataFrame) -> list[str]:
    skip = set(REQUIRED_COLUMNS) | {
        "result",
        "final_result",
        "result_code",
        "source_result_code",
        "is_ok",
        "abs_error_mm",
        "error_mm",
    }
    return [col for col in df.columns if col not in skip]


def join_unique(series: pd.Series):
    values = [str(value) for value in series.dropna().unique()]
    if not values:
        return ""
    if len(values) == 1:
        return values[0]
    return ";".join(sorted(values))


def filter_current_list(df: pd.DataFrame, current_list: str) -> pd.DataFrame:
    if not current_list.strip():
        return df
    currents = {float(item.strip()) for item in current_list.split(",") if item.strip()}
    return df[df["current_ma"].isin(currents)].copy()


def plot_overview_scatter(df: pd.DataFrame, out_path: str, title_prefix: str = "") -> None:
    """全測定点の散布図を保存する。"""
    fig, ax = plt.subplots(figsize=(8, 6))
    marker_map = {"OK": "o", "WARN": "^", "NG": "x", "UNKNOWN": "s"}
    color_col = "abs_error_mm" if df["abs_error_mm"].notna().any() else "current_ma"
    for label, group in df.groupby("final_result", dropna=False):
        scatter_kwargs = {}
        if label != "NG":
            scatter_kwargs["edgecolors"] = "black"
        ax.scatter(
            group["speed_mm_s"],
            group["accel_mm_s2"],
            c=group[color_col],
            marker=marker_map.get(label, "s"),
            s=70,
            label=label,
            cmap="viridis",
            **scatter_kwargs,
        )
    mappable = ax.scatter(df["speed_mm_s"], df["accel_mm_s2"], c=df[color_col], cmap="viridis", alpha=0)
    fig.colorbar(mappable, ax=ax, label=color_col)
    ax.set_xlabel("speed_mm_s")
    ax.set_ylabel("accel_mm_s2")
    ax.set_title(title("Step Loss Sweep Overview", title_prefix))
    ax.legend(title="Result")
    ax.grid(True, alpha=0.3)
    save_figure(fig, out_path)


def plot_error_heatmap_by_current(
    df: pd.DataFrame,
    out_path: str,
    show_cell_labels: bool = True,
    error_scale: str = "linear",
    title_prefix: str = "",
) -> bool:
    """電流ごとの abs_error_mm ヒートマップを保存する。"""
    if not df["abs_error_mm"].notna().any():
        return False
    currents = sorted(df["current_ma"].dropna().unique())
    fig, axes = subplots_for_currents(len(currents), base_width=7.2, base_height=3.4, max_cols=1, constrained=True)
    vmax = df["abs_error_mm"].max()
    positive = df.loc[df["abs_error_mm"] > 0, "abs_error_mm"]
    norm = None
    plot_values = "abs_error_mm"
    if error_scale == "log" and not positive.empty:
        floor = positive.min()
        plot_values = "_plot_abs_error_mm"
        df = df.copy()
        df[plot_values] = df["abs_error_mm"].clip(lower=floor)
        norm = LogNorm(vmin=floor, vmax=max(vmax, floor))

    image = None
    for ax, current in zip(axes, currents):
        pivot = pivot_grid(df[df["current_ma"] == current], plot_values)
        image_kwargs = {"norm": norm} if norm is not None else {"vmin": 0, "vmax": vmax}
        image = ax.imshow(pivot.values, origin="lower", aspect="auto", cmap="magma", **image_kwargs)
        setup_heatmap_axis(ax, pivot, f"current = {fmt_num(current)} mA")
        if show_cell_labels:
            label_heatmap(
                ax,
                pivot,
                fmt="{:.3g}",
                source=pivot_grid(df[df["current_ma"] == current], "abs_error_mm"),
                text_color=lambda value: "white" if vmax > 0 and float(value) / vmax < 0.25 else "black",
            )
    fig.suptitle(title("Position Error Heatmap by Current", title_prefix))
    if image is not None:
        fig.colorbar(image, ax=axes, label="abs_error_mm", fraction=0.025, pad=0.02)
    save_figure(fig, out_path)
    return True


def plot_result_heatmap_by_current(
    df: pd.DataFrame,
    out_path: str,
    show_cell_labels: bool = True,
    title_prefix: str = "",
) -> None:
    """電流ごとの OK/WARN/NG ヒートマップを保存する。"""
    currents = sorted(df["current_ma"].dropna().unique())
    cmap = ListedColormap(["#d95f5f", "#f3d36b", "#6fbf73", "#d8d8d8"])
    norm = BoundaryNorm([-0.25, 0.25, 0.75, 1.25, 1.75], cmap.N)
    fig, axes = subplots_for_currents(len(currents), base_width=7.2, base_height=3.4, max_cols=1, constrained=True)
    image = None
    for ax, current in zip(axes, currents):
        subset = df[df["current_ma"] == current].copy()
        subset["_plot_result"] = subset["result_code"].fillna(1.5)
        pivot = pivot_grid(subset, "_plot_result")
        pivot = pivot.fillna(1.5)
        image = ax.imshow(pivot.values, origin="lower", aspect="auto", cmap=cmap, norm=norm)
        setup_heatmap_axis(ax, pivot, f"current = {fmt_num(current)} mA")
        if show_cell_labels:
            labels = pivot_grid(subset.assign(_label=subset["final_result"]), "_label")
            label_heatmap(ax, labels, fmt="{}")
    fig.suptitle(title("Result Heatmap by Current", title_prefix))
    if image is not None:
        cbar = fig.colorbar(image, ax=axes, ticks=[0, 0.5, 1, 1.5], fraction=0.025, pad=0.02)
        cbar.ax.set_yticklabels(["NG", "WARN", "OK", "unmeasured"])
    save_figure(fig, out_path)


def plot_max_speed_by_current(df: pd.DataFrame, out_path: str, title_prefix: str = "") -> pd.DataFrame:
    """電流ごとの最大安定速度グラフを保存し、集計DFを返す。"""
    ok_df = df[df["final_result"] == "OK"]
    max_speed = ok_df.groupby(["current_ma", "accel_mm_s2"], as_index=False)["speed_mm_s"].max()
    fig, ax = plt.subplots(figsize=(8, 5))
    if max_speed.empty:
        ax.text(0.5, 0.5, "No OK points", ha="center", va="center", transform=ax.transAxes)
    else:
        for accel, group in max_speed.groupby("accel_mm_s2"):
            ax.plot(group["current_ma"], group["speed_mm_s"], marker="o", label=f"accel {fmt_num(accel)}")
        ax.legend(title="accel_mm_s2")
    ax.set_xlabel("current_ma")
    ax.set_ylabel("max OK speed_mm_s")
    ax.set_title(title("Max Stable Speed by Current", title_prefix))
    ax.grid(True, alpha=0.3)
    save_figure(fig, out_path)
    return max_speed


def plot_max_speed_by_accel(df: pd.DataFrame, out_path: str, title_prefix: str = "") -> pd.DataFrame:
    """加速度ごとの最大安定速度グラフを保存し、集計DFを返す。"""
    ok_df = df[df["final_result"] == "OK"]
    max_speed = ok_df.groupby(["accel_mm_s2", "current_ma"], as_index=False)["speed_mm_s"].max()
    fig, ax = plt.subplots(figsize=(8, 5))
    if max_speed.empty:
        ax.text(0.5, 0.5, "No OK points", ha="center", va="center", transform=ax.transAxes)
    else:
        for current, group in max_speed.groupby("current_ma"):
            ax.plot(group["accel_mm_s2"], group["speed_mm_s"], marker="o", label=f"{fmt_num(current)} mA")
        ax.legend(title="current_ma")
    ax.set_xlabel("accel_mm_s2")
    ax.set_ylabel("max OK speed_mm_s")
    ax.set_title(title("Max Stable Speed by Accel", title_prefix))
    ax.grid(True, alpha=0.3)
    save_figure(fig, out_path)
    return max_speed


def plot_error_vs_speed_by_current(df: pd.DataFrame, out_path: str, title_prefix: str = "") -> None:
    """速度に対するずれ量を電流ごとに比較する。"""
    accels = sorted(df["accel_mm_s2"].dropna().unique())
    fig, axes = subplots_for_currents(len(accels), base_width=5.2, base_height=3.6)
    for ax, accel in zip(axes, accels):
        subset = df[df["accel_mm_s2"] == accel]
        for current, group in subset.groupby("current_ma"):
            ax.plot(group["speed_mm_s"], group["abs_error_mm"], marker="o", label=f"{fmt_num(current)} mA")
        ax.set_title(f"accel = {fmt_num(accel)}")
        ax.set_xlabel("speed_mm_s")
        ax.set_ylabel("abs_error_mm")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle(title("Error vs Speed by Current", title_prefix))
    save_figure(fig, out_path)


def calculate_margin_score(df: pd.DataFrame) -> pd.DataFrame:
    """OK条件ごとに周囲セルを見て margin_score を計算する。"""
    df = df.copy()
    df["margin_score"] = np.nan
    by_current = {current: group for current, group in df.groupby("current_ma")}
    for current, group in by_current.items():
        speeds = sorted(group["speed_mm_s"].unique())
        accels = sorted(group["accel_mm_s2"].unique())
        lookup = {
            (row.speed_mm_s, row.accel_mm_s2): row.final_result
            for row in group.itertuples(index=False)
        }
        for idx, row in group.iterrows():
            if row["final_result"] != "OK":
                continue
            measured = 0
            ok_count = 0
            speed_i = speeds.index(row["speed_mm_s"])
            accel_i = accels.index(row["accel_mm_s2"])
            for ds in (-1, 0, 1):
                for da in (-1, 0, 1):
                    if ds == 0 and da == 0:
                        continue
                    ns = speed_i + ds
                    na = accel_i + da
                    if ns < 0 or na < 0 or ns >= len(speeds) or na >= len(accels):
                        continue
                    result = lookup.get((speeds[ns], accels[na]))
                    if result is None:
                        continue
                    measured += 1
                    if result == "OK":
                        ok_count += 1
            df.loc[idx, "margin_score"] = ok_count / measured if measured else 0.0
    return df


def select_recommended_settings(df: pd.DataFrame, min_margin_score: float = 0.5) -> pd.DataFrame:
    """safe / balanced / speed の推奨条件を選ぶ。"""
    ok_df = df[df["final_result"] == "OK"].copy()
    columns = ["rank", "purpose", "speed_mm_s", "accel_mm_s2", "current_ma", "abs_error_mm", "margin_score", "notes"]
    if ok_df.empty:
        return pd.DataFrame(columns=columns)

    candidates = ok_df[ok_df["margin_score"].fillna(0) >= min_margin_score].copy()
    if candidates.empty:
        candidates = ok_df.copy()

    max_speed = candidates["speed_mm_s"].max()
    max_accel = candidates["accel_mm_s2"].max()
    max_current = candidates["current_ma"].max()

    safe_sorted = candidates.sort_values(
        by=["margin_score", "abs_error_mm", "current_ma", "accel_mm_s2", "speed_mm_s"],
        ascending=[False, True, True, True, False],
    )
    balanced = candidates.copy()
    balanced["_balanced_score"] = (
        0.40 * balanced["speed_mm_s"] / max_speed
        + 0.30 * balanced["margin_score"].fillna(0)
        + 0.20 * (1.0 - balanced["abs_error_mm"].fillna(0) / max(balanced["abs_error_mm"].max(), 1e-9))
        + 0.10 * (1.0 - balanced["current_ma"] / max_current)
    )
    balanced_sorted = balanced.sort_values(
        by=["_balanced_score", "current_ma", "accel_mm_s2"],
        ascending=[False, True, True],
    )
    speed_sorted = candidates.sort_values(
        by=["speed_mm_s", "abs_error_mm", "margin_score", "current_ma", "accel_mm_s2"],
        ascending=[False, True, False, True, True],
    )

    picks = [
        make_recommendation(1, "safe", safe_sorted.iloc[0], "安全寄り"),
        make_recommendation(2, "balanced", balanced_sorted.iloc[0], "速度と安定性のバランス"),
        make_recommendation(3, "speed", speed_sorted.iloc[0], "速度重視"),
    ]
    return pd.DataFrame(picks, columns=columns)


def make_recommendation(rank: int, purpose: str, row: pd.Series, notes: str) -> dict:
    return {
        "rank": rank,
        "purpose": purpose,
        "speed_mm_s": row["speed_mm_s"],
        "accel_mm_s2": row["accel_mm_s2"],
        "current_ma": row["current_ma"],
        "abs_error_mm": row["abs_error_mm"],
        "margin_score": row["margin_score"],
        "notes": notes,
    }


def write_markdown_report(
    df: pd.DataFrame,
    recommended_df: pd.DataFrame,
    out_path: str,
    figures_dir: str,
    thresholds: tuple[float, float],
    input_csv: str,
    title: str,
    error_heatmap_generated: bool,
    figure_ext: str,
    max_by_current: pd.DataFrame,
    max_by_accel: pd.DataFrame,
) -> None:
    """Markdownレポートを生成する。"""
    ok_count = int((df["final_result"] == "OK").sum())
    warn_count = int((df["final_result"] == "WARN").sum())
    ng_count = int((df["final_result"] == "NG").sum())
    lines = [
        "# Step Loss Sweep Report",
        "",
        "## Summary",
        f"- Input CSV: `{input_csv}`",
        f"- Total tests: {len(df)}",
        f"- OK count: {ok_count}",
        f"- WARN count: {warn_count}",
        f"- NG count: {ng_count}",
        f"- Speed range: {fmt_num(df['speed_mm_s'].min())} .. {fmt_num(df['speed_mm_s'].max())} mm/s",
        f"- Accel range: {fmt_num(df['accel_mm_s2'].min())} .. {fmt_num(df['accel_mm_s2'].max())} mm/s^2",
        f"- Current range: {fmt_num(df['current_ma'].min())} .. {fmt_num(df['current_ma'].max())} mA",
        f"- Max abs error mm: {fmt_num(df['abs_error_mm'].max()) if df['abs_error_mm'].notna().any() else 'N/A'}",
        f"- Mean abs error mm: {fmt_num(df['abs_error_mm'].mean()) if df['abs_error_mm'].notna().any() else 'N/A'}",
        f"- Median abs error mm: {fmt_num(df['abs_error_mm'].median()) if df['abs_error_mm'].notna().any() else 'N/A'}",
        f"- OK/WARN thresholds: OK <= {thresholds[0]} mm, NG > {thresholds[1]} mm",
        "",
        auto_comment(df),
        "",
        "## Recommended Settings",
        markdown_table_from_df(recommended_df, ["purpose", "speed_mm_s", "accel_mm_s2", "current_ma", "abs_error_mm", "margin_score", "notes"]),
        "",
        "## 1. Overview Scatter",
        f"![Overview]({figures_dir}/{FIGURE_NAMES['overview']}.{figure_ext})",
        "",
        "## 2. Error Heatmap by Current",
    ]
    if error_heatmap_generated:
        lines.append(f"![Error Heatmap]({figures_dir}/{FIGURE_NAMES['error_heatmap']}.{figure_ext})")
    else:
        lines.append("abs_error_mm / error_mm / final_error_mm がないため、ずれ量ヒートマップは生成していません。")
    lines.extend([
        "",
        "## 3. Result Heatmap by Current",
        f"![Result Heatmap]({figures_dir}/{FIGURE_NAMES['result_heatmap']}.{figure_ext})",
        "",
        "## 4. Max Stable Speed by Current",
        f"![Max Stable Speed by Current]({figures_dir}/{FIGURE_NAMES['max_speed_current']}.{figure_ext})",
        "",
        "## 5. Max Stable Speed by Accel",
        f"![Max Stable Speed by Accel]({figures_dir}/{FIGURE_NAMES['max_speed_accel']}.{figure_ext})",
        "",
        "## 6. Error vs Speed",
        f"![Error vs Speed]({figures_dir}/{FIGURE_NAMES['error_speed']}.{figure_ext})",
        "",
        "## Notes",
        "- OK means abs_error_mm is below the OK threshold and no explicit NG was observed.",
        "- WARN means abs_error_mm is above the OK threshold but below the NG threshold.",
        "- NG means abs_error_mm exceeds the NG threshold or explicit NG was observed.",
        "- Recommended settings are selected from measured OK points only.",
        "- Unmeasured cells are not treated as OK.",
        "- Duplicate conditions are aggregated using the worst result and maximum abs_error_mm.",
    ])
    Path(out_path).write_text("\n".join(lines) + "\n")


def auto_comment(df: pd.DataFrame) -> str:
    ok_df = df[df["final_result"] == "OK"]
    if ok_df.empty:
        return "今回の探索では、OK条件は見つかりませんでした。ずれ量とNG領域を確認して探索範囲を下げる必要があります。"
    best = ok_df.sort_values(["speed_mm_s", "margin_score", "abs_error_mm"], ascending=[False, False, True]).iloc[0]
    return (
        f"今回の探索では、速度 {fmt_num(best['speed_mm_s'])} mm/s、"
        f"加速度 {fmt_num(best['accel_mm_s2'])} mm/s^2、"
        f"駆動電流 {fmt_num(best['current_ma'])} mA 付近が速度と安定性の候補です。"
        "OK/NGだけでなくずれ量を確認することで、脱調境界に近い条件を避けやすくなります。"
    )


def pivot_grid(df: pd.DataFrame, value_col: str) -> pd.DataFrame:
    speeds = sorted(df["speed_mm_s"].dropna().unique())
    accels = sorted(df["accel_mm_s2"].dropna().unique())
    pivot = df.pivot_table(index="accel_mm_s2", columns="speed_mm_s", values=value_col, aggfunc="first")
    return pivot.reindex(index=accels, columns=speeds)


def subplots_for_currents(
    count: int,
    base_width: float = 4.2,
    base_height: float = 3.6,
    max_cols: int = 3,
    constrained: bool = False,
):
    cols = min(max_cols, max(1, count))
    rows = math.ceil(count / cols)
    fig, axes = plt.subplots(
        rows,
        cols,
        figsize=(base_width * cols, base_height * rows),
        squeeze=False,
        constrained_layout=constrained,
    )
    axes_list = list(axes.flatten())
    for ax in axes_list[count:]:
        ax.axis("off")
    return fig, axes_list[:count]


def setup_heatmap_axis(ax, pivot: pd.DataFrame, title_text: str) -> None:
    ax.set_title(title_text)
    ax.set_xticks(range(len(pivot.columns)), [fmt_num(value) for value in pivot.columns])
    ax.set_yticks(range(len(pivot.index)), [fmt_num(value) for value in pivot.index])
    ax.set_xlabel("speed_mm_s")
    ax.set_ylabel("accel_mm_s2")


def label_heatmap(ax, pivot: pd.DataFrame, fmt: str = "{:.3g}", source: pd.DataFrame | None = None, text_color="black") -> None:
    source = source if source is not None else pivot
    for y, accel in enumerate(pivot.index):
        for x, speed in enumerate(pivot.columns):
            value = source.loc[accel, speed]
            if pd.isna(value):
                continue
            text = fmt.format(value)
            color = text_color(value) if callable(text_color) else text_color
            ax.text(x, y, text, ha="center", va="center", color=color, fontsize=8)


def save_figure(fig, out_path: str) -> None:
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    if not fig.get_constrained_layout():
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="This figure includes Axes that are not compatible with tight_layout")
            fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def markdown_table_from_df(df: pd.DataFrame, columns: list[str]) -> str:
    if df.empty:
        return "_No measured OK settings available._"
    header = "| " + " | ".join(columns) + " |"
    sep = "| " + " | ".join(["---"] * len(columns)) + " |"
    rows = []
    for _, row in df.iterrows():
        rows.append("| " + " | ".join(format_table_value(row[col]) for col in columns) + " |")
    return "\n".join([header, sep] + rows)


def format_table_value(value) -> str:
    if pd.isna(value):
        return "-"
    if isinstance(value, float):
        return fmt_num(value)
    return str(value)


def title(base: str, prefix: str) -> str:
    return f"{prefix} - {base}" if prefix else base


def fmt_num(value) -> str:
    if pd.isna(value):
        return "-"
    number = float(value)
    if number.is_integer():
        return str(int(number))
    return f"{number:.4f}".rstrip("0").rstrip(".")


if __name__ == "__main__":
    raise SystemExit(main())
