# -*- coding: utf-8 -*-
"""包边验证：两卡扣之间的轮廓段按顶点内角策略扫包边。

- 内角 [15°,180°) 或 180° 平滑过渡 → 铺
- 凹角/极端角 → 不铺
- 卡扣端面区间退化 → 跳过该段并记录可粘贴用例
"""
import sys
sys.path.insert(0, ".")

import tile_generator_v3 as t


def run(name, params, hollow=False, expect_note=""):
    try:
        verts, arcs = t.parse_geometry(params)
        shape, p0_edges, p1_loops, p2_loops, meta = t.build_p0_body(
            verts, arcs, hollow=hollow, params=params)
        line = (
            f"[{name}] valid={meta['isValid']} solids={meta['solidCount']} connected={meta['connected']} "
            f"clips={meta['fusedClipCount']}/{meta['clipCount']} "
            f"圆角={meta['p1CornerApplied']} 包边段={meta['wrapApplied']} sweep={meta['sweepCount']} "
            f"跳过角={meta['wrapSkippedCorners']} 失败角={meta['wrapFailedCorners']} "
            f"nudged={meta['wrapNudged']} vol={meta['volume']}"
        )
        print(line)
        for c in meta["wrapDegenerateCases"]:
            print("    ⚠ 碰扣跳过用例:", c)
        if expect_note:
            print("    " + expect_note)
        return meta
    except Exception as exc:
        print(f"[{name}] FAILED: {exc}")
        return None


if __name__ == "__main__":
    # 直线类
    run("正方形", {"freeSideLengths": [40, 40, 40, 40], "freeAngles": [90]})
    run("正三角形", {"sides": 3, "sideLen": 40})
    run("正六边形", {"sides": 6, "sideLen": 40})
    run("凹四边形210", {"freeSideLengths": [40, 40, 60, 60], "freeAngles": [210]})
    run("小角度四边形", {"freeSideLengths": [54, 37, 52, 40], "freeAngles": [163.1]})
    run("短边凹四边形190", {"freeSideLengths": [30, 30, 30, 40], "freeAngles": [190]})
    run("短边凹四边形210", {"freeSideLengths": [30, 30, 30, 40], "freeAngles": [210]})
    run("长边凹四边形190", {"freeSideLengths": [40, 40, 80, 40], "freeAngles": [190]})
    # 单弧类
    run("正方形单凸弧", {"freeSideLengths": [40, 40, 40, 40], "freeAngles": [90],
                         "freeArcs": {"0": {"radius": 40, "direction": "out"}}})
    run("正方形单凹弧", {"freeSideLengths": [40, 40, 40, 40], "freeAngles": [90],
                         "freeArcs": {"0": {"radius": 40, "direction": "in"}}})
    run("正方形半圆", {"freeSideLengths": [40, 40, 40, 40], "freeAngles": [90],
                       "freeArcs": {"0": {"radius": 20, "direction": "out"}}})
    run("正方形边2凹弧", {"freeSideLengths": [40, 40, 40, 40], "freeAngles": [90],
                          "freeArcs": {"2": {"radius": 40, "direction": "in"}}})
    run("40.5变体", {"freeSideLengths": [40, 40.5, 40, 40], "freeAngles": [90],
                     "freeArcs": {"0": {"radius": 40, "direction": "in"}}})
    # 多弧类
    run("三角形双弧", {"freeSideLengths": [54, 40, 60], "freeAngles": [],
                       "freeArcs": {"0": {"radius": 40, "direction": "in"},
                                    "2": {"radius": 40, "direction": "out"}}})
    run("三角形三弧", {"freeSideLengths": [54, 40, 60], "freeAngles": [],
                       "freeArcs": {"0": {"radius": 40, "direction": "in"},
                                    "1": {"radius": 40, "direction": "out"},
                                    "2": {"radius": 40, "direction": "out"}}})
    run("正方形相邻双凸弧", {"freeSideLengths": [40, 40, 40, 40], "freeAngles": [90],
                            "freeArcs": {"0": {"radius": 40, "direction": "out"},
                                         "1": {"radius": 40, "direction": "out"}}})
    run("正方形四弧", {"freeSideLengths": [40, 40, 40, 40], "freeAngles": [90],
                      "freeArcs": {"0": {"radius": 28.2, "direction": "in"},
                                   "1": {"radius": 28.2, "direction": "in"},
                                   "2": {"radius": 28.2, "direction": "out"},
                                   "3": {"radius": 28.2, "direction": "out"}}})
    run("正方形四弧反向", {"freeSideLengths": [40, 40, 40, 40], "freeAngles": [90],
                          "freeArcs": {"0": {"radius": 28.2, "direction": "out"},
                                       "1": {"radius": 28.2, "direction": "out"},
                                       "2": {"radius": 28.2, "direction": "in"},
                                       "3": {"radius": 28.2, "direction": "in"}}})
    # 多解类
    run("60-80-24-40解1", {"freeSideLengths": [60, 80, 24, 40], "freeAngles": [45], "freeSolution": 0})
    run("62-40解2", {"freeSideLengths": [62, 40, 40, 40], "freeAngles": [90], "freeSolution": 1})
    run("58-106.5解1", {"freeSideLengths": [58, 40, 40, 40], "freeAngles": [106.5], "freeSolution": 0})
    run("58-107.4解2", {"freeSideLengths": [58, 40, 40, 40], "freeAngles": [107.4], "freeSolution": 1})
    run("58-54-40-40解1", {"freeSideLengths": [58, 54, 40, 40], "freeAngles": [80], "freeSolution": 0})
    # 重点回归
    run("四弧56.5", {"freeSideLengths": [56.5, 56.5, 56.5, 56.5], "freeAngles": [90],
                    "freeArcs": {"0": {"radius": 40, "direction": "in"},
                                 "1": {"radius": 40, "direction": "in"},
                                 "2": {"radius": 40, "direction": "out"},
                                 "3": {"radius": 40, "direction": "out"}}})
    run("五边双弧解2", {"freeSideLengths": [61.5, 40, 40, 40, 49.5], "freeAngles": [108, 108],
                         "freeSolution": 1,
                         "freeArcs": {"1": {"radius": 40, "direction": "out"},
                                      "4": {"radius": 65.25, "direction": "in"}}})
    run("三角形40-40-50凸", {"freeSideLengths": [40, 40, 50], "freeAngles": [],
                             "freeArcs": {"1": {"radius": 40, "direction": "out"}}})
    run("三角形40-40-50凹", {"freeSideLengths": [40, 40, 50], "freeAngles": [],
                             "freeArcs": {"1": {"radius": 40, "direction": "in"}}})
    # 用户补充用例
    run("5边凹凸(用户)", {"freeSideLengths": [40, 40, 40, 40, 40], "freeAngles": [108, 108],
                          "freeArcs": {"0": {"radius": 40, "direction": "in"},
                                       "1": {"radius": 40, "direction": "in"},
                                       "2": {"radius": 40, "direction": "out"},
                                       "3": {"radius": 40, "direction": "in"},
                                       "4": {"radius": 40, "direction": "out"}}})
    run("4边凹凸(用户)", {"freeSideLengths": [40, 40, 40, 40], "freeAngles": [90],
                          "freeArcs": {"0": {"radius": 40, "direction": "in"},
                                       "1": {"radius": 40, "direction": "out"},
                                       "2": {"radius": 40, "direction": "in"},
                                       "3": {"radius": 40, "direction": "out"}}})
    run("7边形(用户)", {"freeSideLengths": [40, 40, 40, 40, 40, 40, 40],
                        "freeAngles": [128.6, 128.6, 128.6, 25.7]})
    run("5边形凹弧R80(用户)", {"freeSideLengths": [80, 80, 80, 80, 80],
                                "freeAngles": [108, 108], "freeSolution": 1,
                                "freeArcs": {"4": {"radius": 80, "direction": "in"}}})
    print("完成")
