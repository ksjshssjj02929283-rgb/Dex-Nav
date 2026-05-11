# nav2_smoother docs 协作规则

## 文档语言

- 本目录文档默认使用中文。
- 命令、路径、类名、参数名、topic 名和 action 名可保留原文。

## 维护范围

- `README.md` 记录 `nav2_smoother` 的平滑器原理、参数、碰撞检测关系和排查建议。
- 源码根目录 `README.md` 可保留为包简介；涉及实现细节和调试说明时，优先更新本目录 `README.md`。

## 同步规则

- 修改 `SimpleSmoother`、`SavitzkyGolaySmoother`、`SmootherServer`、碰撞检查逻辑、插件参数或配置示例时，同步更新 `docs/README.md`。
- 修改本文档维护规则时，同步更新本文件。
- 如果新增 smoother 插件，需要在 `docs/README.md` 中补充其算法原理、参数、碰撞检测行为和适用场景。

## 验证要求

- 文档更新后，优先用 `rg` 或静态阅读确认类名、参数名、常量名与源码一致。
- 不主动执行 `colcon build`；如确需构建验证，先询问用户。
