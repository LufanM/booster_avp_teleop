import SwiftUI

@main
struct VisionProTeleopApp: App {
    @StateObject private var model = 🥽AppModel()
    @Environment(\.scenePhase) var scenePhase // 新增场景阶段监听
    @Environment(\.dismissImmersiveSpace) var dismissImmersiveSpace
    @Environment(\.dismissWindow) var dismissWindow // 新增窗口关闭操作符
    @Environment(\.openWindow) var openWindow


    var body: some Scene {
        WindowGroup(id: "mainWindow") {
            ContentView()
                .environmentObject(model) // ✅ 注入环境对象
        }
        .windowResizability(.contentSize)
        
        WindowGroup(id: "videoStreamWindow") {
            VideoStreamWindow()
                .environmentObject(model) // ✅ 注入
        }
        .windowResizability(.contentSize)
        
        ImmersiveSpace(id: "immersiveSpace") {
            🌐RealityView(model: model) // ✅ 注入
        }
        .onChange(of: scenePhase) { _, newPhase in
            Task { @MainActor in
                  switch newPhase {
                  case .background:
                      // 强制关闭所有非主窗口
                      await dismissImmersiveSpace()
                      dismissWindow(id: "videoStreamWindow")
                      print("强制关闭视频窗口")
                      model.resetAllStates()
                  case .active:
                      if !model.isMainWindowOpen {
                          print("强制打开主窗口")
                          // 强制打开主窗口
                          await self.openMainWindow()
                      }
                  default: break
                  }
              }
        }
    }
    init() {
        🧑HeadTrackingComponent.registerComponent()
        🧑HeadTrackingSystem.registerSystem()
    }
    
    // MARK: - 强制打开主窗口（仅 visionOS）
    private func openMainWindow() async {
        print("Connected scenes: \(UIApplication.shared.connectedScenes)")

        #if os(visionOS)
//        if let windowScene = UIApplication.shared.connectedScenes.first as? UIWindowScene {
//            let window = UIWindow(windowScene: windowScene)
//            window.rootViewController = UIHostingController(
//                rootView: ContentView().environmentObject(model)
//            )
//            window.isHidden = false
//            window.windowLevel = .normal + 1 // 置顶显示
//        }
        openWindow(id: "mainWindow")
        dismissWindow(id: "videoStreamWindow")
        #endif
    }
}


