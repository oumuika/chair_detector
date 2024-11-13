# chair_detector

椅子検出用パッケージ

```mermaid
flowchart TB;
A[/scanのサブスクライブ] -- クラスタのサイズでフィルタ --> B[椅子の脚の検出]
B --> C[];
C -- 晴れている --> D{屋外で BBQ};
C -- 雨が降っている --> E{屋内で BBQ};
D --> F[テントの設営];
E --> G[施設の方に許可を得る];
F --> H[BBQ 開始]
G --> H[BBQ 開始]
```
