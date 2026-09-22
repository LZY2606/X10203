# 关节坐标册 (Joint Atlas)

审阅 URDF、独立版本化标定变换与现场关节快照，回答“任意两个 frame 之间的姿态由哪些定义组成”。

## 运行

```bash
./gradlew build -x test          # 构建
./gradlew test                   # 测试
./gradlew run --args='--host 127.0.0.1 --port 5263'   # 启动
# 打开 http://127.0.0.1:5263
```

首次启动会写入演示数据（含固定关节环路、 mimic 关节、标定集与快照）到 `jointatlas.db`（SQLite）。

## 结构

- `src/main/kotlin/atlas/xml/Xml.kt` — 保真 XML：保留未知元素/属性/注释与原始顺序，可往返导出
- `src/main/kotlin/atlas/urdf/Urdf.kt` — link/joint/mimic/limit/origin/inertial/geometry 解析
- `src/main/kotlin/atlas/urdf/Patch.kt` — 获批补丁应用（只改已知属性）与版本差异
- `src/main/kotlin/atlas/kinematics/Engine.kt` — frame 图、多路径候选、环路残差、最小矛盾边集、mimic 链/环、标定时间/序列号生效
- `src/main/kotlin/atlas/store/Store.kt` — SQLite：URDF 版本历史、标定集、快照、草案（版本号并发控制）
- `src/main/kotlin/atlas/server/` — Ktor REST + 单页 UI（图、三维坐标轴、残差、差异、冻结比较）

## 主要 API

- `POST /api/urdf` 导入；`GET /api/urdf/{id}/export` 导出（仅含获批补丁）
- `GET /api/query?from=&to=&at=&serial=&units=rad|deg&joints=j1=0.5` 坐标查询（完整链/关节值/矩阵/来源/候选）
- `GET /api/cycles` 环路残差与最小矛盾边集；`GET /api/graph` link/joint 图
- `POST /api/calibrations` 标定集（robotSerial + validFrom/validTo，端点含边界标记）
- `POST /api/snapshots` 冻结快照；`GET /api/compare?snapshotId=&calibrationId=&urdfId=` 对比另一套标定
- `POST /api/drafts` → `approve` → `apply`（baseVersion 冲突返回 409）
