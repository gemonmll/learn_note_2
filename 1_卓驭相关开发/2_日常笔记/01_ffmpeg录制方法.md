可以，FFmpeg 很适合做这两件事：

1. **录制屏幕视频**
2. **从视频里抽关键帧 / 场景变化帧 / 固定间隔帧**

FFmpeg 官方文档里提供了不同平台的抓屏设备：

* Windows 常用 `gdigrab`。([FFmpeg][1])
* Linux X11 常用 `x11grab`。([FFmpeg][1])
* macOS 常用 `avfoundation`。([FFmpeg][1])

下面我直接给你能用的命令。

---

## 一、录制视频

### Windows 录屏

录整个桌面：

```bash
ffmpeg -f gdigrab -framerate 15 -i desktop -c:v libx264 -preset veryfast -crf 23 screen.mp4
```

FFmpeg 官方文档里 `gdigrab` 的示例就是用 `-i desktop` 抓整个桌面，也支持指定区域和窗口标题。([FFmpeg][1])

录指定区域，比如左上角开始的 1280x720：

```bash
ffmpeg -f gdigrab -framerate 15 -offset_x 0 -offset_y 0 -video_size 1280x720 -i desktop -c:v libx264 -preset veryfast -crf 23 screen.mp4
```

录某个窗口：

```bash
ffmpeg -f gdigrab -framerate 15 -i title="Visual Studio Code" -c:v libx264 -preset veryfast -crf 23 vscode.mp4
```

### Linux 录屏

录整个 X11 桌面：

```bash
ffmpeg -f x11grab -video_size 1920x1080 -framerate 15 -i :0.0 -c:v libx264 -preset veryfast -crf 23 screen.mp4
```

FFmpeg 官方示例里 `x11grab` 就是这种写法，也支持 `:0.0+10,20` 这种偏移抓取。([FFmpeg][1])

录指定区域：

```bash
ffmpeg -f x11grab -video_size 1280x720 -framerate 15 -i :0.0+100,100 -c:v libx264 -preset veryfast -crf 23 area.mp4
```

### macOS 录屏

先列设备：

```bash
ffmpeg -f avfoundation -list_devices true -i ""
```

FFmpeg 官方文档说明 `avfoundation` 可以先列出设备，再用 `-i "video:audio"` 选择。([FFmpeg][1])

录屏示例：

```bash
ffmpeg -f avfoundation -framerate 15 -i "1:none" -c:v libx264 -preset veryfast -crf 23 screen.mp4
```

---

## 二、从视频抽帧

### 1）固定时间间隔抽帧

每秒抽 1 张：

```bash
ffmpeg -i screen.mp4 -vf fps=1 frames/frame_%05d.png
```

每 2 秒 1 张：

```bash
ffmpeg -i screen.mp4 -vf fps=1/2 frames/frame_%05d.png
```

这个最简单，但会有很多重复帧。

---

### 2）按关键帧抽取

只抽视频本身的关键帧：

```bash
ffmpeg -skip_frame nokey -i screen.mp4 -vsync 0 frames/key_%05d.png
```

FFmpeg 官方文档明确给了 `-skip_frame nokey` 提取 keyframe 的示例，并说明 `-vsync 0` 可以避免重复输出。([FFmpeg][1])

不过要注意：

**视频编码里的关键帧，不一定等于“代码发生变化的关键时刻”。**
它只是编码器定义的 I 帧。

---

### 3）按“画面变化”抽帧

这个更适合代码视频。

```bash
ffmpeg -i screen.mp4 -vf "select='gt(scene,0.03)'" -vsync vfr frames/scene_%05d.png
```

FFmpeg 的 `select` 过滤器支持按 `scene` 值筛帧，官方示例里给了 `select='gt(scene,0.4)'`，并提到阈值通常在 `0.3` 到 `0.5` 是常见起点。([FFmpeg][1])

但代码视频和电影不一样，代码变化通常很小，所以建议你把阈值调低一些，例如：

* `0.01`
* `0.02`
* `0.03`

比如更敏感一些：

```bash
ffmpeg -i screen.mp4 -vf "select='gt(scene,0.015)'" -vsync vfr frames/scene_%05d.png
```

---

### 4）每隔 N 帧抽 1 张

比如每 100 帧抽 1 张：

```bash
ffmpeg -i screen.mp4 -vf "select='not(mod(n,100))'" -vsync vfr frames/sample_%05d.png
```

官方文档也给了这个例子。([FFmpeg][1])

---

## 三、最适合“代码恢复”的抽帧方案

你这个目标不是普通视频截图，而是**从代码视频恢复源码**，所以我建议这样做：

### 方案 A：先录得清楚

录制时尽量用：

* `15 fps` 就够了
* 分辨率尽量高，最好 `1920x1080`
* 字体放大一点
* 用 `libx264`
* `-crf 18~23`

例如 Windows：

```bash
ffmpeg -f gdigrab -framerate 15 -i desktop -c:v libx264 -preset veryfast -crf 20 screen.mp4
```

### 方案 B：先按场景变化抽

```bash
ffmpeg -i screen.mp4 -vf "select='gt(scene,0.015)'" -vsync vfr frames/scene_%05d.png
```

### 方案 C：再补固定间隔抽帧

因为代码滚动时，`scene` 不一定稳定抓到：

```bash
ffmpeg -i screen.mp4 -vf fps=1/2 frames/fixed_%05d.png
```

然后把两批帧一起拿去做 OCR / 代码整理。

---

## 四、直接给你一个实用组合

### Windows 录 VSCode 窗口

```bash
ffmpeg -f gdigrab -framerate 15 -i title="Visual Studio Code" -c:v libx264 -preset veryfast -crf 20 vscode.mp4
```

### 从录好的视频里按变化抽帧

```bash
ffmpeg -i vscode.mp4 -vf "select='gt(scene,0.015)'" -vsync vfr frames/scene_%05d.png
```

### 再每 2 秒补一张

```bash
ffmpeg -i vscode.mp4 -vf fps=1/2 frames/fixed_%05d.png
```

### 如果你还想抽 keyframe

```bash
ffmpeg -skip_frame nokey -i vscode.mp4 -vsync 0 frames/key_%05d.png
```

---

## 五、几个很重要的经验

### 1. 代码视频不要录太高帧率

录代码 10 到 15 fps 通常就够了。
太高只会让文件变大，抽帧后重复内容更多。

### 2. “关键帧”不等于“代码有变化的帧”

要恢复代码，通常：

* `scene` 抽帧比 keyframe 更有用
* 固定间隔抽帧是补漏

### 3. 最好录到这些区域

为了后面恢复目录结构，录屏时尽量带上：

* 左侧文件树
* 顶部标签页
* 底部终端
* 编辑区

这样后面更容易恢复项目结构。

---

## 六、给你一个最推荐的命令组合

如果你是 **Windows**，我建议先这样：

```bash
ffmpeg -f gdigrab -framerate 15 -i title="Visual Studio Code" -c:v libx264 -preset veryfast -crf 20 vscode.mp4
ffmpeg -i vscode.mp4 -vf "select='gt(scene,0.015)'" -vsync vfr frames/scene_%05d.png
ffmpeg -i vscode.mp4 -vf fps=1/2 frames/fixed_%05d.png
```

这套对“代码还原”已经比较实用了。([FFmpeg][1])

你要的话，我下一条可以直接给你一套 **Windows 批处理版**，做到“一键录制 + 一键抽帧”。

[1]: https://ffmpeg.org/ffmpeg-all.html "      ffmpeg Documentation
"
