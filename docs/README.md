# dexe_mobile_nav 文档

本目录记录仓库级维护说明，当前补充内容聚焦 `package.xml` 元数据同步。

## package.xml 元数据同步

- 根目录 `package.xml` 作为仓库级元数据来源，当前版本号为 `0.4.0`。
- 各功能包 `package.xml` 已同步使用 `0.4.0` 版本号。
- 各功能包 maintainer 统一为 `dexforce <dexforce@dexforce.com>`。
- 各功能包保留原有 `description`，不强制改为根包的 CI 说明。
- Dexforce 自研包使用 `Proprietary` license。
- `mobile_navigation` 下的 Nav2 上游包保留原有 Apache、BSD、MIT、LGPL 等 license，不追加 `Proprietary`。

## 验证方式

本次修改未执行构建。可通过 XML 解析和文本搜索验证包清单字段一致性。
