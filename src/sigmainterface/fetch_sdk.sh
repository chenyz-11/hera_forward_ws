#!/usr/bin/env bash
# ============================================================
# 拉取 SigmaSDK 依赖到 sigma_sdk/（对应 requirements.txt 条目）
# 用法: ./fetch_sdk.sh
# 说明: 取 sdk_261014 分支最新 commit；目录已存在则跳过
# ============================================================
set -euo pipefail
cd "$(dirname "$0")"

REPO="https://gitee.com/beijing-daxiang-airlines/SigmaSDKDemo.git"
BRANCH="sdk_261014"
DEST="sigma_sdk"

if [ -d "$DEST" ]; then
    echo "已存在 ${DEST}/，跳过拉取（如需更新请先删除该目录再执行）"
    exit 0
fi

echo ">> git clone -b ${BRANCH} --depth 1 ${REPO} ${DEST}"
git clone -b "${BRANCH}" --depth 1 "${REPO}" "${DEST}"

echo ">> 完成，当前 commit: $(git -C "${DEST}" log -1 --oneline)"
