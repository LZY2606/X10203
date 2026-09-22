# 关节坐标册 (Joint Coordinate Book)

审阅 URDF、独立版本化标定变换与现场关节快照,回答"任意两个 frame 之间的姿态由哪些定义组成"。

## 运行

```bash
./gradlew build -x test          # 构建
./gradlew test                   # 测试
./gradlew run --args='--host 127.0.0.1 --port 5263'   # 打开 http://127.0.0.1:5263
```

首次启动自动播种演示机器人 `SN-001`(含 mimic 关节、固定关节闭环标定、重复声明、度单位标定)。

## 设计要点

- **URDF 往返保真**: 以 DOM 保存原始文档,未知元素/属性/顺序原样保留;导出仅应用**已获批**补丁,标定绝不写回 URDF。
- **运动学图**: link/joint 与标定统一成边;枚举所有简单路径并保留并列候选及相互误差,不按遍历顺序取舍。
- **环路**: 生成基本环并计算闭环残差(平移+旋转);对不一致环求最小矛盾边集(最小命中集,优先指向重复声明的标定边)。
- **标定生效**: 按机器人序列号 + [validFrom, validTo] 闭区间;边界时点报 `active:boundary-inclusive`,缺查询时刻/缺关节值均有明确状态。
- **并发控制**: 编辑草案携带基版本号,批准时与最新版本比对,过期返回 409。
- **快照**: 可冻结现场关节值,再与另一时刻(另一套标定)对比姿态与环残差。

## API 概览

| 端点 | 说明 |
| --- | --- |
| `GET /api/state?serial=&time=` | 图、边、环残差、矛盾边集、版本、快照、草案 |
| `GET /api/query?serial=&from=&to=&time=&snapshot=` | 全部候选链、关节值、单位、矩阵、来源 |
| `POST /api/import` | 导入 URDF(版本递增) |
| `POST /api/calibration` | 新增标定版本(独立版本化) |
| `POST /api/draft` / `/api/draft/{id}/approve` | 草案与乐观并发批准 |
| `POST /api/snapshot` | 冻结现场快照 |
| `GET /api/export?serial=` | 导出 XML(仅获批补丁,未知内容保留) |
| `GET /api/diff?serial=&v1=&v2=` | URDF 版本结构差异 |
| `GET /api/compare?snapshot=&from=&to=&time=` | 冻结快照 vs 另一时刻标定 |
