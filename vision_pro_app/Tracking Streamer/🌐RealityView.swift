import SwiftUI
import RealityKit
import ARKit
import UIKit

struct 🌐RealityView: View {	
    @ObservedObject var model: 🥽AppModel
    static let attachmentID: String = "resultLabel"

    var body: some View {
        RealityView { content, attachments in
            // 在沉浸式空间中创建3D平面用于渲染视频流
//            let videoPlane = createVideoPlane()
//            videoPlane.components.set(🧑HeadTrackingComponent())
//            content.add(videoPlane)
            
            
            let resultLabelEntity = attachments.entity(for: Self.attachmentID)!
            resultLabelEntity.components.set(🧑HeadTrackingComponent())
            resultLabelEntity.name = 🧩Name.resultLabel
            
        } update: { content, attachments in
            // 更新逻辑
              if let videoPlane = content.entities.first(where: { $0.name == "VideoScreen" }) as? ModelEntity {
                  updateTexture(for: videoPlane)
              }
            // 获取头部姿态
//            let headTransform = DataManager.shared.latestHandTrackingData.Head
//            print("头部位姿态：\(headTransform)")
//            let angle = Float.random(in: -15...15) * .pi / 180
//            let headTransform = simd_float4x4(simd_quatf(angle: angle, axis: [0, 1, 0]))

            // 初始姿态参数
//            let positionOffset = SIMD3<Float>(0, 1, -2) // 位置偏移
//            let rotateX = simd_quatf(angle: .pi/2, axis: [1, 0, 0]) // X轴旋转90度
//            let rotateY = simd_quatf(angle: -.pi/6, axis: [0, 1, 0]) // Y轴旋转-30度
//            
//            // 构造初始变换矩阵（先旋转后平移）
//            let combinedRotation = rotateY * rotateX
//            var initialTransform = matrix_identity_float4x4
//            initialTransform *= simd_float4x4(combinedRotation) // 应用旋转
//            initialTransform.columns.3 = SIMD4<Float>(positionOffset.x,  // 应用平移
//                                                     positionOffset.y,
//                                                     positionOffset.z,
//                                                     1)
//            
//            // 计算最终变换（头部姿态 * 初始偏移）
//            let finalTransform = headTransform * initialTransform
//            let finalTransform =  initialTransform
//
//            
//            // 应用到视频平面
//            if let videoPlane = content.entities.first(where: { $0.name == "VideoScreen" }) {
//                videoPlane.transform = Transform(matrix: finalTransform)
//                
//                // 保持平面与头部相对朝向（可选）
//                videoPlane.orientation = combinedRotation
//            }
        } attachments: {
            Attachment(id: Self.attachmentID) {
            }
        }
        .gesture(
            TapGesture()
                .targetedToAnyEntity()
        )
        .task { self.model.run() }
        .task { await self.model.processDeviceAnchorUpdates() }
        .task { self.model.startserver() }
        .task(priority: .low) { await self.model.processReconstructionUpdates() }
    }
    
    // MARK: - 视频平面创建， base是在3D世界原点，初始位姿与水平面一致
    private func createVideoPlane() -> ModelEntity {
        let plane = ModelEntity(
            mesh: .generatePlane(width: 1.6, depth: 0.9),
            materials: [SimpleMaterial(color: .white, isMetallic: false)]
        )
        plane.name = "VideoScreen"
        plane.position = [0, 1, -2]
        let rotateX = simd_quatf(angle: .pi/2, axis: [1, 0, 0]) // 绕 X 轴 90 度
        let rotateY = simd_quatf(angle: -.pi/6, axis: [0, 1, 0]) // 绕 Y 轴 45 度
        plane.orientation = rotateY * rotateX // 组合旋转

        return plane
    }
    
    // MARK: - 纹理更新
    private func updateTexture(for entity: ModelEntity) {
        guard let jpegData = model.currentJpegData,
              let uiImage = UIImage(data: jpegData),
              let cgImage = uiImage.cgImage else { return }
        
        do {
            let texture = try TextureResource.generate(
                from: cgImage,
                withName: nil,
                options: .init(semantic: .color))
            var material = SimpleMaterial()
            material.baseColor = .texture(texture)
            entity.model?.materials = [material]
        } catch {
            print("纹理创建失败: \(error)")
        }
    }
}
		
