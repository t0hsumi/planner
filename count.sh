#!/bin/bash

# ベースディレクトリ
BASE_DIR="./test"

# サブディレクトリのリスト
SUBDIRS=("$BASE_DIR"/*)

# plannerの実行ファイルのパス
PLANNER="./planner"

# 各サブディレクトリに対して実行
for SUBDIR in "${SUBDIRS[@]}"
do
  # サブディレクトリのパス
  SUBDIR_PATH="$SUBDIR"

  # domain.pddl のパス
  DOMAIN_PDDL="$SUBDIR_PATH/domain.pddl"

  # サブディレクトリ内のタスクファイルのリスト
  TASK_FILES=("$SUBDIR_PATH"/*.pddl)

  # 各タスクファイルに対して planner を実行
  for TASK_PDDL in "${TASK_FILES[@]}"
  do
    # plannerの実行
    $PLANNER $DOMAIN_PDDL $TASK_PDDL
  done
done
