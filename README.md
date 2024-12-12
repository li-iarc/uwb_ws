# uwb_ws
UWB program for tracking

## 共通架構
- 部份程式有版本2(基本上就是檔名多了2),差異補充在程式的開頭註解中
- 執行時首先開啟terminal,進入工作區,執行以下指令
```bash
cd uwb_ws
```
- 初次使用時需進行編譯
```bash
colcon build
```

- 載入環境設定檔
```bash
. install/setup.bash 
```
- 接著就可以執行下方節點,一個terminal執行一個,每次開新的terminal都要載入環境設定檔
- 若有修改檔案,則在儲存後在工作區資料夾執行以下指令以進行編譯
```bash
colcon build
```
  
### UWBReceiver
- 用於接收UWB資訊，再將資訊傳出
- 接收時會進行濾波,將誤差過大的數值過濾掉
- 範例為執行使用編號0、1、2號的UWB時的指令
```bash
ros2 run uwb UWBReceiver 0 1 2
```

### UWBDataAggregator
- 用於統整UWBReceiver接收到的資訊,再將資訊傳出
- 範例為執行使用編號0、1、2號的UWB時的指令
```bash
ros2 run uwb UWBDataAggregator 0 1 2
```

### UWBCalculator
- 用於計算從UWBDataAggregator接收到的資訊,再將計算結果(如使用者座標與角度)傳出
- 計算結果會再次進行濾波
- 若UWB裝設位置有更動,需更改程式內的點座標
- 範例為執行指令
```bash
ros2 run uwb UWBCalculator
```

### UWBPlotter
- 用於將UWBCalculator的計算結果以圖顯示
- 若UWB裝設位置有更動,需更改程式內的點座標
- 範例為執行指令
```bash
ros2 run uwb UWBPlotter
```


## 專案差異說明
### uwb
- 較為基礎的架構,以三個uwb組成

### uwb4
- 以四個uwb組成,計算時以三個一組計算出四個結果再取平均得出目標座標
- 結果較前者穩定,但隨著距離與雜訊的增加會有明顯誤差

### uwb5
- 透過新的UWB外殼設計提高精確度,不過僅限前方180度,為此改以五個UWB朝不同方向再結合進行計算
- 以五個uwb組成,不過計算時僅用到3個
- 目前未完成,程式應改也還只有寫三個UWB,原訂預計以數值最小的三個進行計算(還不確定此方法是否有漏洞),若位於車體左右兩側則藉由車體旋轉以維持使用在在車子前方
