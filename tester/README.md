演练测试自动化：已由用户于 2026-09-11 解除禁令，可由 `python -m codes.gui_autopilot` 自动完成（代启模拟器→登录→关闭公告→点「开始问题N演练测试」→等接口就绪→跑 practice 命令）。

正式测试红线不变：任何情况下不得触碰界面上任何"正式测试"入口；工具对含「正式」的按钮只记录不点击，点击目标必须与白名单「开始问题N演练测试」逐字相等。

技术要点：模拟器是 WebView2 应用（窗口标题「无线电干扰源环境模拟器」），UIA 控件树惰性加载，需先向 Chrome_RenderWidgetHostHWND 发 WM_GETOBJECT(OBJID_CLIENT) 激活；WebView2 调试端口方案不可行（应用覆盖了 WEBVIEW2_ADDITIONAL_BROWSER_ARGUMENTS，且用户要求不改环境变量，已撤销）。
