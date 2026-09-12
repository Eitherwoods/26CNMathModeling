演练测试自动化：已由用户于 2026-09-11 解除禁令，可由 `python -m codes.gui_autopilot` 自动完成（代启模拟器→登录→关闭公告→点「开始问题N演练测试」→等接口就绪→跑 practice 命令）。

**测试策略（2026-09-12 起）**：所有测试都在本模拟器的"演练模式"进行——本地只保留一两套正确性单元测试，通过后直接跑线上演练，不再做本地离线仿真/基准。

正式测试红线不变：任何情况下不得触碰界面上任何"正式测试"入口；工具对含「正式」的按钮只记录不点击，点击目标必须与白名单「开始问题N演练测试」逐字相等。

技术要点：模拟器是 WebView2 应用（窗口标题「无线电干扰源环境模拟器」），UIA 控件树惰性加载，需先向 Chrome_RenderWidgetHostHWND 发 WM_GETOBJECT(OBJID_CLIENT) 激活；WebView2 调试端口方案不可行（应用覆盖了 WEBVIEW2_ADDITIONAL_BROWSER_ARGUMENTS，且用户要求不改环境变量，已撤销）。

**本地训练（2026-09-12 新增）**：不上服务器、不打开官方模拟器的本地演练见
[READMElocal.md](READMElocal.md)——`codes/local_simulator.py` 按官方演练模式逆向复刻
（随机数、场景生成、示向度噪声、计费与 HTTP 协议一致），用于策略离线迭代与批量自测；
逆向结论与证据见 [reverse/NOTES.md](reverse/NOTES.md)。
