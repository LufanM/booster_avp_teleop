import SwiftUI
import CoreLocation
import UIKit
import SystemConfiguration.CaptiveNetwork
import RealityKit

struct ContentView: View {
    @Environment(\.openImmersiveSpace) var openImmersiveSpace
    @Environment(\.dismissWindow) var dismissWindow
    @Environment(\.openWindow) var openWindow
    @EnvironmentObject var model: 🥽AppModel
    @State private var isTransitioning = false

    var body: some View {
        VStack(spacing: 32) {
            HStack(spacing: 28) {
                Image(.graph2)
                    .resizable()
                    .aspectRatio(contentMode: .fit)
                    .frame(width: 1200)
                    .clipShape(.rect(cornerRadius: 24))
            }
            Text("You're on IP address [\(getIPAddress())]")
                .font(.largeTitle.weight(.medium))
            
            // 视频窗口切换按钮
            Button {
                model.isVideoWindowOpen.toggle()
                if model.isVideoWindowOpen {
                    openWindow(id: "videoStreamWindow")
                }
            } label: {
                Text(model.isVideoWindowOpen ? "Close Video Window" : "Open Video Window")
                    .font(.largeTitle)
            }
            
            // 启动沉浸式空间按钮
            Button {
                Task {
                    model.isTransitioning = true
                    if await openImmersiveSpace(id: "immersiveSpace") == .opened {
//                        model.isImmersiveSpaceActive = true
                        dismissWindow() // 关闭主窗口
                    }
                    model.isTransitioning = false
                }
            } label: {
                Text("Start")
                    .font(.largeTitle)
                    .padding(.vertical, 12)
                    .padding(.horizontal, 4)
            }
            .disabled(model.isTransitioning) // 防止重复点击
            .onAppear {
                self.model.isMainWindowOpen = true
                }
            .onDisappear {
                self.model.isMainWindowOpen = false
            }
        }
    }
    
    func getIPAddress() -> String {
        var address: String?
        var ifaddr: UnsafeMutablePointer<ifaddrs>? = nil
        if getifaddrs(&ifaddr) == 0 {
            var ptr = ifaddr
            while ptr != nil {
                defer { ptr = ptr?.pointee.ifa_next }
                
                guard let interface = ptr?.pointee else { return "" }
                let addrFamily = interface.ifa_addr.pointee.sa_family
                if addrFamily == UInt8(AF_INET) || addrFamily == UInt8(AF_INET6) {
                    
                    // wifi = ["en0"]
                    // wired = ["en2", "en3", "en4"]
                    // cellular = ["pdp_ip0","pdp_ip1","pdp_ip2","pdp_ip3"]
                    
                    let name: String = String(cString: (interface.ifa_name))
                    if  name == "en0" || name == "en2" || name == "en3" || name == "en4" || name == "pdp_ip0" || name == "pdp_ip1" || name == "pdp_ip2" || name == "pdp_ip3" {
                        var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                        getnameinfo(interface.ifa_addr, socklen_t((interface.ifa_addr.pointee.sa_len)), &hostname, socklen_t(hostname.count), nil, socklen_t(0), NI_NUMERICHOST)
                        address = String(cString: hostname)
                    }
                }
            }
            freeifaddrs(ifaddr)
        }
        return address ?? ""
    }
    
    func getWiFiName() -> String? {
        var ssid: String?
        
        if let interfaces = CNCopySupportedInterfaces() as NSArray? {
            for interface in interfaces {
                if let interfaceInfo = CNCopyCurrentNetworkInfo(interface as! CFString) as NSDictionary? {
                    ssid = interfaceInfo[kCNNetworkInfoKeySSID as String] as? String
                    break
                }
            }
        }
        
        return ssid
    }
}

struct VideoStreamWindow: View {
    @EnvironmentObject var model: 🥽AppModel
    @Environment(\.dismissWindow) var dismissWindow
    @Environment(\.openWindow) var openWindow
    @State private var currentImage: UIImage? // 存储当前视频帧

    var body: some View {
        ZStack {
            // 背景颜色
            Color.gray.opacity(0.2)
              .ignoresSafeArea()
              .overlay(
                  RoundedRectangle(cornerRadius: 12)
                      .stroke(Color.blue, lineWidth: 4) // 蓝色边框
              )
        
            // 根据状态显示内容
            if model.isVideoStreaming {
                // 视频流内容
                if let image = currentImage {
                    Image(uiImage: image)
                        .resizable()
                        .aspectRatio(contentMode: .fit)
                        .frame(maxWidth: 1200, maxHeight: 600)
                } else {
                    Button("Start Video Stream") {
                        model.startVideoStreaming()
                    }
                    .font(.title)
                    .padding()
                    .background(Color.white.opacity(0.8))
                    .cornerRadius(8)
                }
            } else {
                // 未启动时显示按钮
                Button("Start Video Stream") {
                    model.startVideoStreaming() // ✅ 触发视频流启动
                }
                .font(.title)
                .padding()
                .background(Color.white.opacity(0.8))
                .cornerRadius(8)
            }
        }
        .frame(width: 1280, height: 720) // 固定窗口尺寸
        .onChange(of: model.isVideoWindowOpen) { open in
            if !open {
                dismissWindow()
            }
        }
        .onChange(of: model.currentJpegData) { _ in
            // 数据更新时刷新图像
            updateCurrentImage()
        }
        .onDisappear {
            print("关闭视频窗口")
            model.isVideoWindowOpen = false
        }
    }
    
    // MARK: - 利用UIKit渲染，更新当前帧
    private func updateCurrentImage() {
        guard let data = model.currentJpegData else {
            currentImage = nil
            return
        }
        DispatchQueue.global(qos: .userInitiated).async {
            guard let image = UIImage(data: data) else { return }
            DispatchQueue.main.async {
                self.currentImage = image
            }
        }
    }
    
    // MARK: - 视频平面创建,3D平面base是窗口，利用3D平面进行渲染
    private func createVideoPlane() -> ModelEntity {
        let plane = ModelEntity(
            mesh: .generatePlane(width: 0.6, depth: 0.3),
            materials: [SimpleMaterial(color: .white, isMetallic: false)]
        )
        plane.name = "VideoScreen"
        plane.position = [0, 0, 0]
        plane.orientation = simd_quatf(angle: .pi/0, axis: [0, 1, 0]) // ✅ 绕 y 轴旋转
        return plane
    }
    
    // MARK: - 更新材质
    private func updateTexture(for entity: ModelEntity) {
        guard let jpegData = model.currentJpegData,
              let uiImage = UIImage(data: jpegData),
              let cgImage = uiImage.cgImage,
              let texture = try? TextureResource.generate(from: cgImage, options: .init(semantic: .color)) else { return }
        if uiImage == nil {
            print("无法根据数据创建 UIImage 对象")
        }
 
        var material = SimpleMaterial()
        material.baseColor = .texture(texture)
        entity.model?.materials = [material]
    }
}
