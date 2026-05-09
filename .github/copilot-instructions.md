# Copilot 指令来源说明

工程规范、代码风格、调试流程、踩坑经验等指令通过 VS Code settings.json 直接从知识库加载：

- 工程行为模式：`NoteSpace/.claude/memory/identities/robot_engineer/work.md`
- 决策表达风格：`NoteSpace/.claude/memory/identities/robot_engineer/persona.md`

蒸馏更新后知识库文件即时生效，无需手动同步。

---

## 启动协议（知识库对齐）

每次对话开始，立即读取以下文件，将当前工程与知识库规则、历史决策、踩坑经验自动对齐：

```
~/文档/Nutstore Files/NoteSpace/.claude/memory/MEMORY.md
  └── 主索引：身份激活规则、活跃项目列表
~/文档/Nutstore Files/NoteSpace/.claude/memory/identities/robot_engineer/robot_engineer_identity.md
  └── 当前目标、踩坑备忘、活跃项目
```

若本次对话已绑定项目（`/continue-project`）→ 额外读取该项目的 `_MOC.md`。
若任务涉及导航知识 → 读取 `NoteSpace/10. 机器人导航/4. 导航/_MOC.md` 按需展开。

**默认激活身份**：`robot_engineer`（SLAM / 导航 / ROS2 / Nav2）
