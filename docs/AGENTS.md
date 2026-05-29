# docs 目录协作规则

## 维护范围

- 本目录用于记录仓库级文档和维护约定。
- 修改仓库级元数据、运行方式、配置策略或跨包规则时，应同步更新本目录文档。

## package.xml 维护约定

- 根目录 `package.xml` 的 `version` 作为仓库级版本来源。
- 功能包 `package.xml` 应同步仓库级版本号和 maintainer，当前 maintainer 为 `dexforce <dexforce@dexforce.com>`。
- 功能包 `description` 保持包自身说明，不复用根包的 CI 描述。
- Dexforce 自研包使用 `Proprietary` license。
- `mobile_navigation` 下的 Nav2 上游包保留原有 Apache、BSD、MIT、LGPL 等 license，不追加 `Proprietary`，不要删除上游 license 声明。

## 验证约束

- 默认不执行 `colcon build` 或其他构建命令。
- 修改 `package.xml` 后优先使用 XML 解析、`rg` 文本检查等非构建方式验证。

## nav2_theta_star_planner 维护约束

- 修改 `ThetaStar::isSafe()` 时，需要同时关注普通邻居扩展和 LOS 检查两个重载版本，避免两处安全判定不一致。
- 当前 Theta* 规划约束为只允许原始 costmap 自由栅格通行；膨胀层代价栅格应视为不可通行。
