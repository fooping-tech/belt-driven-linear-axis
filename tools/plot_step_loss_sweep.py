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
    "velocity_elapsed": "07_velocity_elapsed_by_speed",
    "elapsed_variability": "08_elapsed_variability_by_speed",
    "elapsed_histogram": "09_elapsed_histogram",
    "pc_wait_elapsed": "10_pc_wait_elapsed_by_speed",
    "firmware_motion_elapsed": "11_firmware_motion_elapsed_by_speed",
    "expected_ideal_elapsed": "12_expected_ideal_by_speed",
    "update_gap_elapsed_error": "13_update_gap_vs_elapsed_error",
    "sg_elapsed_comparison": "14_sg_elapsed_comparison",
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
    variability_df = summarize_variability(classified_df)

    raw_out = out_dir / "raw_results.csv"
    classified_df.to_csv(raw_out, index=False)
    variability_df.to_csv(out_dir / "variability_stats.csv", index=False)

    figure_paths = {
        "overview": figures_dir / f"{FIGURE_NAMES['overview']}.{args.format}",
        "error_heatmap": figures_dir / f"{FIGURE_NAMES['error_heatmap']}.{args.format}",
        "result_heatmap": figures_dir / f"{FIGURE_NAMES['result_heatmap']}.{args.format}",
        "max_speed_current": figures_dir / f"{FIGURE_NAMES['max_speed_current']}.{args.format}",
        "max_speed_accel": figures_dir / f"{FIGURE_NAMES['max_speed_accel']}.{args.format}",
        "error_speed": figures_dir / f"{FIGURE_NAMES['error_speed']}.{args.format}",
        "velocity_elapsed": figures_dir / f"{FIGURE_NAMES['velocity_elapsed']}.{args.format}",
        "elapsed_variability": figures_dir / f"{FIGURE_NAMES['elapsed_variability']}.{args.format}",
        "elapsed_histogram": figures_dir / f"{FIGURE_NAMES['elapsed_histogram']}.{args.format}",
        "pc_wait_elapsed": figures_dir / f"{FIGURE_NAMES['pc_wait_elapsed']}.{args.format}",
        "firmware_motion_elapsed": figures_dir / f"{FIGURE_NAMES['firmware_motion_elapsed']}.{args.format}",
        "expected_ideal_elapsed": figures_dir / f"{FIGURE_NAMES['expected_ideal_elapsed']}.{args.format}",
        "update_gap_elapsed_error": figures_dir / f"{FIGURE_NAMES['update_gap_elapsed_error']}.{args.format}",
        "sg_elapsed_comparison": figures_dir / f"{FIGURE_NAMES['sg_elapsed_comparison']}.{args.format}",
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
    velocity_elapsed_generated = plot_velocity_elapsed_by_speed(classified_df, str(figure_paths["velocity_elapsed"]), args.title)
    elapsed_variability_generated = plot_elapsed_variability_by_speed(variability_df, str(figure_paths["elapsed_variability"]), args.title)
    elapsed_histogram_generated = plot_elapsed_histogram(classified_df, str(figure_paths["elapsed_histogram"]), args.title)
    pc_wait_elapsed_generated = plot_metric_by_speed(classified_df, "test2_forward_pc_wait_elapsed_ms", str(figure_paths["pc_wait_elapsed"]), "PC Wait Elapsed vs Speed", "pc_wait_elapsed_ms", args.title)
    firmware_motion_elapsed_generated = plot_metric_by_speed(classified_df, "test2_forward_firmware_motion_elapsed_ms", str(figure_paths["firmware_motion_elapsed"]), "Firmware Motion Elapsed vs Speed", "firmware_motion_elapsed_ms", args.title)
    expected_ideal_elapsed_generated = plot_metric_by_speed(classified_df, "test2_expected_ideal_ms", str(figure_paths["expected_ideal_elapsed"]), "Expected Ideal Elapsed vs Speed", "expected_ideal_ms", args.title)
    update_gap_elapsed_error_generated = plot_update_gap_vs_elapsed_error(classified_df, str(figure_paths["update_gap_elapsed_error"]), args.title)
    sg_elapsed_comparison_generated = plot_sg_elapsed_comparison(classified_df, str(figure_paths["sg_elapsed_comparison"]), args.title)

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
        velocity_elapsed_generated=velocity_elapsed_generated,
        variability_df=variability_df,
        elapsed_variability_generated=elapsed_variability_generated,
        elapsed_histogram_generated=elapsed_histogram_generated,
        pc_wait_elapsed_generated=pc_wait_elapsed_generated,
        firmware_motion_elapsed_generated=firmware_motion_elapsed_generated,
        expected_ideal_elapsed_generated=expected_ideal_elapsed_generated,
        update_gap_elapsed_error_generated=update_gap_elapsed_error_generated,
        sg_elapsed_comparison_generated=sg_elapsed_comparison_generated,
    )

    print(f"Report: {out_dir / 'report.md'}")
    print(f"Raw results: {raw_out}")
    print(f"Variability stats: {out_dir / 'variability_stats.csv'}")
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
    if "test2_forward_elapsed_ms" in df.columns:
        df["test2_forward_elapsed_ms"] = pd.to_numeric(df["test2_forward_elapsed_ms"], errors="coerce")
    for col in (
        "test2_forward_pc_wait_elapsed_ms",
        "test2_forward_firmware_motion_elapsed_ms",
        "test2_expected_ideal_ms",
        "test2_expected_firmware_model_ms",
        "test2_elapsed_error_ms",
        "test2_elapsed_error_ratio",
        "max_update_gap_us",
        "sg_enabled",
    ):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
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
    early_limit_ok = (
        has_error
        & (df["abs_error_mm"] <= ok_error_threshold)
        & (df.get("final_limit_timing", "") == "EARLY_LIMIT")
    )
    result_code.loc[early_limit_ok] = 1.0
    combined.loc[early_limit_ok] = 1.0

    explicit_ng = (result_code == 0.0) & ~early_limit_ok
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
    key_columns = condition_key_columns(df)
    for key, group in df.groupby(key_columns, dropna=False):
        key_values = key if isinstance(key, tuple) else (key,)
        key_map = dict(zip(key_columns, key_values))
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
            "result_code": result_code,
            "final_result": result_label(result_code),
            "is_ok": result_code == 1.0,
            "abs_error_mm": abs_error,
            "error_mm": error_mm,
            "sample_count": len(group),
        }
        row.update(key_map)
        for col in optional_columns(group):
            row[col] = join_unique(group[col])
        grouped.append(row)
    return pd.DataFrame(grouped).sort_values(key_columns).reset_index(drop=True)


def summarize_variability(df: pd.DataFrame) -> pd.DataFrame:
    """同一条件のrepeat結果からelapsedと成功率のばらつきを集計する。"""
    if "test2_forward_elapsed_ms" not in df.columns:
        return pd.DataFrame()

    key_columns = condition_key_columns(df)
    working = df.copy()
    working["test2_forward_elapsed_ms"] = pd.to_numeric(working["test2_forward_elapsed_ms"], errors="coerce")
    working["pass_flag"] = working["final_result"].eq("OK")

    grouped_rows: list[dict[str, object]] = []
    for key, group in working.groupby(key_columns, dropna=False, sort=True):
        key_values = key if isinstance(key, tuple) else (key,)
        row: dict[str, object] = dict(zip(key_columns, key_values))
        elapsed = group["test2_forward_elapsed_ms"].dropna()
        total = len(group)
        pass_count = int(group["pass_flag"].sum())
        row.update({
            "sample_count": total,
            "pass_count": pass_count,
            "fail_count": total - pass_count,
            "success_rate": pass_count / total if total else np.nan,
            "elapsed_mean_ms": elapsed.mean() if not elapsed.empty else np.nan,
            "elapsed_std_ms": elapsed.std(ddof=1) if len(elapsed) > 1 else 0.0 if len(elapsed) == 1 else np.nan,
            "elapsed_min_ms": elapsed.min() if not elapsed.empty else np.nan,
            "elapsed_p05_ms": elapsed.quantile(0.05) if not elapsed.empty else np.nan,
            "elapsed_median_ms": elapsed.median() if not elapsed.empty else np.nan,
            "elapsed_p95_ms": elapsed.quantile(0.95) if not elapsed.empty else np.nan,
            "elapsed_max_ms": elapsed.max() if not elapsed.empty else np.nan,
        })
        if not elapsed.empty:
            row["elapsed_range_ms"] = row["elapsed_max_ms"] - row["elapsed_min_ms"]
            mean = float(row["elapsed_mean_ms"])
            row["elapsed_cv_pct"] = float(row["elapsed_std_ms"]) / mean * 100.0 if mean else np.nan
        else:
            row["elapsed_range_ms"] = np.nan
            row["elapsed_cv_pct"] = np.nan
        grouped_rows.append(row)

    if not grouped_rows:
        return pd.DataFrame()
    return pd.DataFrame(grouped_rows).sort_values(key_columns).reset_index(drop=True)


def condition_key_columns(df: pd.DataFrame) -> list[str]:
    columns = list(REQUIRED_COLUMNS)
    if "chop_mode" in df.columns:
        columns.append("chop_mode")
    return columns


def optional_columns(df: pd.DataFrame) -> list[str]:
    skip = set(condition_key_columns(df)) | {
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
    marker_map = {"OK": "o", "WARN": "^", "NG": "x", "UNKNOWN": "s"}
    color_col = "abs_error_mm" if df["abs_error_mm"].notna().any() else "current_ma"
    currents = sorted(df["current_ma"].dropna().unique())
    fig, axes = subplots_for_currents(len(currents), base_width=5.8, base_height=4.2, max_cols=2, constrained=True)
    modes = sorted(df["chop_mode"].dropna().unique()) if "chop_mode" in df.columns else []
    offsets = chop_offsets(df["speed_mm_s"], modes)
    cmap = "viridis"
    color_values = df[color_col]
    vmin = 0 if color_col == "abs_error_mm" else color_values.min()
    vmax = color_values.max() if color_values.notna().any() else 1
    mappable = None

    present_results: set[str] = set()
    for ax, current in zip(axes, currents):
        subset = df[df["current_ma"] == current].copy()
        for label, group in subset.groupby("final_result", dropna=False):
            present_results.add(str(label))
            marker = marker_map.get(label, "s")
            edgecolors = "black" if label != "NG" else None
            x_values = group["speed_mm_s"] + group.apply(lambda row: offsets.get(str(row.get("chop_mode", "")), 0.0), axis=1)
            known = group[color_col].notna()
            if known.any():
                scatter = ax.scatter(
                    x_values[known],
                    group.loc[known, "accel_mm_s2"],
                    c=group.loc[known, color_col],
                    marker=marker,
                    s=80,
                    cmap=cmap,
                    vmin=vmin,
                    vmax=vmax,
                    alpha=0.9,
                    edgecolors=edgecolors,
                )
                mappable = scatter
            if (~known).any():
                ax.scatter(
                    x_values[~known],
                    group.loc[~known, "accel_mm_s2"],
                    color="#bdbdbd",
                    marker=marker,
                    s=80,
                    alpha=0.85,
                    edgecolors=edgecolors,
                )
        ax.set_xlabel("speed_mm_s")
        ax.set_ylabel("accel_mm_s2")
        ax.set_title(f"current = {fmt_num(current)} mA")
        ax.grid(True, alpha=0.3)
        ax.set_xticks(sorted(df["speed_mm_s"].dropna().unique()))

    if mappable is not None:
        fig.colorbar(mappable, ax=axes, label=color_col, fraction=0.025, pad=0.02)
    fig.suptitle(title("Step Loss Sweep Overview by Current", title_prefix))
    add_overview_legend(axes[0], marker_map, present_results)
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
    facets = heatmap_facets(df)
    fig, axes = subplots_for_currents(len(facets), base_width=7.2, base_height=3.4, max_cols=1, constrained=True)
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
    for ax, facet in zip(axes, facets):
        subset = facet_subset(df, facet)
        pivot = pivot_grid(subset, plot_values)
        image_kwargs = {"norm": norm} if norm is not None else {"vmin": 0, "vmax": vmax}
        image = ax.imshow(pivot.values, origin="lower", aspect="auto", cmap="magma", **image_kwargs)
        setup_heatmap_axis(ax, pivot, facet_title(facet))
        if show_cell_labels:
            label_heatmap(
                ax,
                pivot,
                fmt="{:.3g}",
                source=pivot_grid(subset, "abs_error_mm"),
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
    facets = heatmap_facets(df)
    cmap = ListedColormap(["#d95f5f", "#f3d36b", "#6fbf73", "#d8d8d8"])
    norm = BoundaryNorm([-0.25, 0.25, 0.75, 1.25, 1.75], cmap.N)
    fig, axes = subplots_for_currents(len(facets), base_width=7.2, base_height=3.4, max_cols=1, constrained=True)
    image = None
    for ax, facet in zip(axes, facets):
        subset = facet_subset(df, facet).copy()
        subset["_plot_result"] = subset["result_code"].fillna(1.5)
        pivot = pivot_grid(subset, "_plot_result")
        pivot = pivot.fillna(1.5)
        image = ax.imshow(pivot.values, origin="lower", aspect="auto", cmap=cmap, norm=norm)
        setup_heatmap_axis(ax, pivot, facet_title(facet))
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


def plot_velocity_elapsed_by_speed(df: pd.DataFrame, out_path: str, title_prefix: str = "") -> bool:
    """速度指示値に対するtest2の5mm移動完了時間を描く。"""
    if "test2_forward_elapsed_ms" not in df.columns:
        return False

    plot_df = df[df["test2_forward_elapsed_ms"].notna()].copy()
    if plot_df.empty:
        return False

    group_columns = ["current_ma", "accel_mm_s2"]
    if "chop_mode" in plot_df.columns:
        group_columns.append("chop_mode")
    if "microsteps" in plot_df.columns:
        group_columns.append("microsteps")

    fig, ax = plt.subplots(figsize=(8, 5))
    for key, group in plot_df.groupby(group_columns, dropna=False):
        group = group.sort_values("speed_mm_s")
        label = velocity_elapsed_label(group_columns, key)
        ax.plot(group["speed_mm_s"], group["test2_forward_elapsed_ms"], marker="o", label=label)

    ax.set_title(title("Commanded Speed vs test2 Forward Elapsed", title_prefix))
    ax.set_xlabel("commanded speed_mm_s")
    ax.set_ylabel("elapsed_ms for test2 command=5")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    save_figure(fig, out_path)
    return True


def plot_metric_by_speed(
    df: pd.DataFrame,
    metric: str,
    out_path: str,
    plot_title: str,
    ylabel: str,
    title_prefix: str = "",
) -> bool:
    if metric not in df.columns:
        return False
    plot_df = df[df[metric].notna()].copy()
    if plot_df.empty:
        return False

    group_columns = ["current_ma", "accel_mm_s2"]
    if "chop_mode" in plot_df.columns:
        group_columns.append("chop_mode")
    if "microsteps" in plot_df.columns:
        group_columns.append("microsteps")
    if "sg_enabled" in plot_df.columns and plot_df["sg_enabled"].nunique(dropna=True) > 1:
        group_columns.append("sg_enabled")

    fig, ax = plt.subplots(figsize=(8, 5))
    for key, group in plot_df.groupby(group_columns, dropna=False):
        group = group.sort_values("speed_mm_s")
        label = velocity_elapsed_label(group_columns, key)
        ax.plot(group["speed_mm_s"], group[metric], marker="o", label=label)

    ax.set_title(title(plot_title, title_prefix))
    ax.set_xlabel("speed_mm_s")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    save_figure(fig, out_path)
    return True


def plot_update_gap_vs_elapsed_error(df: pd.DataFrame, out_path: str, title_prefix: str = "") -> bool:
    required = {"max_update_gap_us", "test2_elapsed_error_ms"}
    if not required.issubset(df.columns):
        return False
    plot_df = df[df["max_update_gap_us"].notna() & df["test2_elapsed_error_ms"].notna()].copy()
    if plot_df.empty:
        return False

    fig, ax = plt.subplots(figsize=(8, 5))
    color = plot_df["speed_mm_s"] if "speed_mm_s" in plot_df.columns else None
    scatter = ax.scatter(plot_df["max_update_gap_us"], plot_df["test2_elapsed_error_ms"], c=color, cmap="viridis", s=70, alpha=0.85)
    if color is not None:
        fig.colorbar(scatter, ax=ax, label="speed_mm_s")
    ax.axhline(0, color="#6b7280", linewidth=1)
    ax.set_title(title("Max Update Gap vs Elapsed Error", title_prefix))
    ax.set_xlabel("max_update_gap_us")
    ax.set_ylabel("elapsed_error_ms")
    ax.grid(True, alpha=0.3)
    save_figure(fig, out_path)
    return True


def plot_sg_elapsed_comparison(df: pd.DataFrame, out_path: str, title_prefix: str = "") -> bool:
    required = {"sg_enabled", "speed_mm_s"}
    metric = "test2_forward_firmware_motion_elapsed_ms"
    fallback_metric = "test2_forward_pc_wait_elapsed_ms"
    if metric not in df.columns and fallback_metric in df.columns:
        metric = fallback_metric
    if metric not in df.columns or not required.issubset(df.columns):
        return False
    plot_df = df[df[metric].notna() & df["sg_enabled"].notna()].copy()
    if plot_df.empty or plot_df["sg_enabled"].nunique(dropna=True) < 2:
        return False

    group_columns = ["sg_enabled", "accel_mm_s2"]
    if "current_ma" in plot_df.columns and plot_df["current_ma"].nunique(dropna=True) > 1:
        group_columns.append("current_ma")
    if "chop_mode" in plot_df.columns and plot_df["chop_mode"].nunique(dropna=True) > 1:
        group_columns.append("chop_mode")

    fig, ax = plt.subplots(figsize=(8, 5))
    for key, group in plot_df.groupby(group_columns, dropna=False):
        group = group.sort_values("speed_mm_s")
        label = velocity_elapsed_label(group_columns, key)
        ax.plot(group["speed_mm_s"], group[metric], marker="o", label=label)

    ax.set_title(title("SG Enabled vs Disabled Elapsed", title_prefix))
    ax.set_xlabel("speed_mm_s")
    ax.set_ylabel(metric)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    save_figure(fig, out_path)
    return True


def plot_elapsed_variability_by_speed(df: pd.DataFrame, out_path: str, title_prefix: str = "") -> bool:
    """速度に対する平均elapsedと標準偏差を加速度別に表示する。"""
    required = {"speed_mm_s", "accel_mm_s2", "elapsed_mean_ms", "elapsed_std_ms"}
    if df.empty or not required.issubset(df.columns):
        return False

    plot_df = df[df["elapsed_mean_ms"].notna()].copy()
    if plot_df.empty:
        return False

    group_columns = ["accel_mm_s2"]
    if "current_ma" in plot_df.columns and plot_df["current_ma"].nunique(dropna=False) > 1:
        group_columns.append("current_ma")
    if "chop_mode" in plot_df.columns and plot_df["chop_mode"].nunique(dropna=False) > 1:
        group_columns.append("chop_mode")
    if "microsteps" in plot_df.columns and plot_df["microsteps"].nunique(dropna=False) > 1:
        group_columns.append("microsteps")

    fig, ax = plt.subplots(figsize=(8, 5))
    for key, group in plot_df.groupby(group_columns, dropna=False):
        group = group.sort_values("speed_mm_s")
        label = velocity_elapsed_label(group_columns, key)
        ax.errorbar(
            group["speed_mm_s"],
            group["elapsed_mean_ms"],
            yerr=group["elapsed_std_ms"].fillna(0),
            marker="o",
            capsize=3,
            linewidth=1.8,
            label=label,
        )

    ax.set_title(title("Elapsed Mean and StdDev by Speed", title_prefix))
    ax.set_xlabel("commanded speed_mm_s")
    ax.set_ylabel("test2 elapsed_ms mean +/- stddev")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    save_figure(fig, out_path)
    return True


def plot_elapsed_histogram(df: pd.DataFrame, out_path: str, title_prefix: str = "") -> bool:
    """test2_forward_elapsed_msの分布を加速度別ヒストグラムで表示する。"""
    if "test2_forward_elapsed_ms" not in df.columns:
        return False

    plot_df = df[df["test2_forward_elapsed_ms"].notna()].copy()
    if plot_df.empty:
        return False

    fig, ax = plt.subplots(figsize=(8, 5))
    accel_values = sorted(plot_df["accel_mm_s2"].dropna().unique()) if "accel_mm_s2" in plot_df.columns else []
    bins = min(40, max(10, int(math.sqrt(len(plot_df))) + 5))
    if accel_values:
        for accel in accel_values:
            group = plot_df[plot_df["accel_mm_s2"] == accel]
            ax.hist(
                group["test2_forward_elapsed_ms"],
                bins=bins,
                alpha=0.35,
                label=f"accel {fmt_num(accel)}",
                edgecolor="white",
                linewidth=0.5,
            )
        ax.legend(fontsize=8, title="accel_mm_s2")
    else:
        ax.hist(plot_df["test2_forward_elapsed_ms"], bins=bins, color="#4c78a8", alpha=0.8, edgecolor="white")

    ax.set_title(title("test2 Forward Elapsed Histogram", title_prefix))
    ax.set_xlabel("elapsed_ms for test2 command=5")
    ax.set_ylabel("sample count")
    ax.grid(True, axis="y", alpha=0.3)
    save_figure(fig, out_path)
    return True


def velocity_elapsed_label(columns: list[str], key) -> str:
    values = key if isinstance(key, tuple) else (key,)
    parts: list[str] = []
    for column, value in zip(columns, values):
        if column == "current_ma":
            parts.append(f"{fmt_num(value)} mA")
        elif column == "accel_mm_s2":
            parts.append(f"accel {fmt_num(value)}")
        elif column == "microsteps":
            parts.append(f"1/{fmt_num(value)}")
        elif column == "chop_mode":
            parts.append(str(value))
        elif column == "sg_enabled":
            parts.append(f"SG {fmt_num(value)}")
        else:
            parts.append(f"{column}={fmt_num(value) if isinstance(value, (int, float)) else value}")
    return ", ".join(parts)


def calculate_margin_score(df: pd.DataFrame) -> pd.DataFrame:
    """OK条件ごとに周囲セルを見て margin_score を計算する。"""
    df = df.copy()
    df["margin_score"] = np.nan
    group_columns = ["current_ma"]
    if "chop_mode" in df.columns:
        group_columns.append("chop_mode")
    for _group_key, group in df.groupby(group_columns, dropna=False):
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
    columns = ["rank", "purpose", "speed_mm_s", "accel_mm_s2", "current_ma"]
    if "chop_mode" in df.columns:
        columns.append("chop_mode")
    columns.extend(["abs_error_mm", "margin_score", "notes"])
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
    recommendation = {
        "rank": rank,
        "purpose": purpose,
        "speed_mm_s": row["speed_mm_s"],
        "accel_mm_s2": row["accel_mm_s2"],
        "current_ma": row["current_ma"],
        "abs_error_mm": row["abs_error_mm"],
        "margin_score": row["margin_score"],
        "notes": notes,
    }
    if "chop_mode" in row.index:
        recommendation["chop_mode"] = row["chop_mode"]
    return recommendation


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
    velocity_elapsed_generated: bool,
    variability_df: pd.DataFrame,
    elapsed_variability_generated: bool,
    elapsed_histogram_generated: bool,
    pc_wait_elapsed_generated: bool,
    firmware_motion_elapsed_generated: bool,
    expected_ideal_elapsed_generated: bool,
    update_gap_elapsed_error_generated: bool,
    sg_elapsed_comparison_generated: bool,
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
        markdown_table_from_df(recommended_df, recommendation_columns(recommended_df)),
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
        "## 7. Commanded Speed vs Elapsed",
    ])
    if velocity_elapsed_generated:
        lines.append(f"![Commanded Speed vs Elapsed]({figures_dir}/{FIGURE_NAMES['velocity_elapsed']}.{figure_ext})")
    else:
        lines.append("test2_forward_elapsed_ms がないため、速度指示値ごとの elapsed_ms グラフは生成していません。")
    lines.extend([
        "",
        "## 8. Elapsed Variability",
    ])
    if elapsed_variability_generated:
        lines.append(f"![Elapsed Variability]({figures_dir}/{FIGURE_NAMES['elapsed_variability']}.{figure_ext})")
    else:
        lines.append("repeat結果または test2_forward_elapsed_ms が不足しているため、ばらつきグラフは生成していません。")
    lines.extend([
        "",
        "### Largest elapsed variation",
        markdown_table_from_df(variability_rank_table(variability_df), variability_columns()),
        "",
        "Full condition-level statistics are written to `variability_stats.csv`.",
        "",
        "## 9. Elapsed Histogram",
    ])
    if elapsed_histogram_generated:
        lines.append(f"![Elapsed Histogram]({figures_dir}/{FIGURE_NAMES['elapsed_histogram']}.{figure_ext})")
    else:
        lines.append("test2_forward_elapsed_ms がないため、elapsed_ms のヒストグラムは生成していません。")

    lines.extend([
        "",
        "## 10. Timing Decomposition",
    ])
    timing_figures = [
        (pc_wait_elapsed_generated, "PC Wait Elapsed", "pc_wait_elapsed"),
        (firmware_motion_elapsed_generated, "Firmware Motion Elapsed", "firmware_motion_elapsed"),
        (expected_ideal_elapsed_generated, "Expected Ideal Elapsed", "expected_ideal_elapsed"),
        (update_gap_elapsed_error_generated, "Max Update Gap vs Elapsed Error", "update_gap_elapsed_error"),
        (sg_elapsed_comparison_generated, "SG Enabled vs Disabled Elapsed", "sg_elapsed_comparison"),
    ]
    for generated, label, key in timing_figures:
        lines.append("")
        lines.append(f"### {label}")
        if generated:
            lines.append(f"![{label}]({figures_dir}/{FIGURE_NAMES[key]}.{figure_ext})")
        else:
            lines.append("必要な列が不足しているため、このグラフは生成していません。")
    lines.extend(timing_diagnosis(df))

    lines.extend([
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


def timing_diagnosis(df: pd.DataFrame) -> list[str]:
    lines = ["", "### Timing Diagnosis", ""]
    pc = numeric_series(df, "test2_forward_pc_wait_elapsed_ms")
    fw = numeric_series(df, "test2_forward_firmware_motion_elapsed_ms")
    post = numeric_series(df, "test2_forward_post_motion_before_complete_ms")
    err = numeric_series(df, "test2_elapsed_error_ms")
    gap = numeric_series(df, "max_update_gap_us")

    pc_minus_fw = (pc - fw).dropna() if not pc.empty and not fw.empty else pd.Series(dtype=float)
    summary_rows = [
        ["PC待ち時間由来", diagnosis_label(pc_minus_fw, threshold=20.0), metric_summary(pc_minus_fw, "pc_wait - firmware_motion ms")],
        ["ファームsummary出力由来", diagnosis_label(post.dropna(), threshold=20.0), metric_summary(post.dropna(), "post_motion_before_complete ms")],
        ["SG/TMC UARTブロック由来", sg_blocking_diagnosis(df, err, gap), "sg_enabled別比較とmax_update_gap_us相関を確認"],
        ["motion profile離散化由来", diagnosis_label(err.abs().dropna(), threshold=10.0), metric_summary(err.dropna(), "firmware_motion - expected_model ms")],
    ]
    lines.append(markdown_table_rows(["判定項目", "判定", "根拠"], summary_rows))
    return lines


def numeric_series(df: pd.DataFrame, column: str) -> pd.Series:
    if column not in df.columns:
        return pd.Series(dtype=float)
    return pd.to_numeric(df[column].astype(str).str.split(";").str[0], errors="coerce")


def diagnosis_label(values: pd.Series, threshold: float) -> str:
    if values.empty:
        return "UNKNOWN"
    median_abs = values.abs().median()
    if pd.isna(median_abs):
        return "UNKNOWN"
    return "LIKELY" if median_abs >= threshold else "UNLIKELY"


def metric_summary(values: pd.Series, label: str) -> str:
    if values.empty:
        return f"{label}: NA"
    return f"{label}: median={values.median():.3f}, max={values.max():.3f}"


def markdown_table_rows(headers: list[str], rows: list[list[str]]) -> str:
    table = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        table.append("| " + " | ".join(str(value) for value in row) + " |")
    return "\n".join(table)


def sg_blocking_diagnosis(df: pd.DataFrame, err: pd.Series, gap: pd.Series) -> str:
    if "sg_enabled" not in df.columns:
        return "UNKNOWN"
    sg = numeric_series(df, "sg_enabled")
    if sg.dropna().nunique() < 2:
        return "UNKNOWN"
    metric = numeric_series(df, "test2_forward_firmware_motion_elapsed_ms")
    if metric.empty:
        metric = numeric_series(df, "test2_forward_pc_wait_elapsed_ms")
    enabled = metric[sg == 1].dropna()
    disabled = metric[sg == 0].dropna()
    if enabled.empty or disabled.empty:
        return "UNKNOWN"
    delta = enabled.median() - disabled.median()
    if delta >= 20.0:
        return "LIKELY"
    if not err.empty and not gap.empty and len(err.dropna()) >= 3:
        corr = err.corr(gap)
        if not pd.isna(corr) and corr >= 0.6:
            return "POSSIBLE"
    return "UNLIKELY"


def variability_columns() -> list[str]:
    return [
        "speed_mm_s",
        "accel_mm_s2",
        "sample_count",
        "pass_count",
        "success_rate",
        "elapsed_mean_ms",
        "elapsed_std_ms",
        "elapsed_min_ms",
        "elapsed_max_ms",
        "elapsed_range_ms",
        "elapsed_cv_pct",
    ]


def variability_rank_table(df: pd.DataFrame, limit: int = 25) -> pd.DataFrame:
    if df.empty:
        return pd.DataFrame(columns=variability_columns())
    ranked = df.copy()
    for column in variability_columns():
        if column not in ranked.columns:
            ranked[column] = np.nan
    return ranked.sort_values(
        ["elapsed_std_ms", "elapsed_range_ms", "speed_mm_s"],
        ascending=[False, False, True],
    ).head(limit)[variability_columns()]


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


def heatmap_facets(df: pd.DataFrame) -> list[dict[str, object]]:
    columns = ["current_ma"]
    if "chop_mode" in df.columns:
        columns.append("chop_mode")
    facets: list[dict[str, object]] = []
    for values, _group in df.groupby(columns, dropna=False, sort=True):
        values_tuple = values if isinstance(values, tuple) else (values,)
        facets.append(dict(zip(columns, values_tuple)))
    return facets


def facet_subset(df: pd.DataFrame, facet: dict[str, object]) -> pd.DataFrame:
    subset = df
    for column, value in facet.items():
        subset = subset[subset[column] == value]
    return subset


def facet_title(facet: dict[str, object]) -> str:
    current = fmt_num(facet["current_ma"])
    if "chop_mode" in facet:
        return f"current = {current} mA, chop = {facet['chop_mode']}"
    return f"current = {current} mA"


def chop_offsets(speed_series: pd.Series, modes: list[str]) -> dict[str, float]:
    if not modes:
        return {}
    speeds = sorted(speed_series.dropna().unique())
    if len(speeds) >= 2:
        min_step = min(b - a for a, b in zip(speeds, speeds[1:]) if b > a)
    else:
        min_step = 1.0
    spread = min_step * 0.16
    center = (len(modes) - 1) / 2.0
    return {str(mode): (index - center) * spread for index, mode in enumerate(modes)}


def add_overview_legend(ax, marker_map: dict[str, str], present_results: set[str]) -> None:
    from matplotlib.lines import Line2D

    result_handles = [
        Line2D([0], [0], marker=marker, color="black", linestyle="None", markersize=7, label=label)
        for label, marker in marker_map.items()
        if label in present_results
    ]
    ax.legend(handles=result_handles, title="Result", loc="best", frameon=True)


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


def recommendation_columns(df: pd.DataFrame) -> list[str]:
    columns = ["purpose", "speed_mm_s", "accel_mm_s2", "current_ma"]
    if "chop_mode" in df.columns:
        columns.append("chop_mode")
    columns.extend(["abs_error_mm", "margin_score", "notes"])
    return columns


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
