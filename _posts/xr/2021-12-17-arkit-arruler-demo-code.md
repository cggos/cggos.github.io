---
title: 基于ARKit的AR尺子Demo代码分析
tags:
  - ARKit
categories:
  - XR
key: xr-arkit-arruler-demo-code
abbrlink: f4d3f6b7
date: 2021-12-17 00:00:00
---

* Code: https://github.com/levantAJ/Measure

<p align="center">
  <img src="/img/post/arkit/arkit_arruler_demo_code.png" style="width:50%;"/>
</p>

- UIViewController
  ```swift
  final class ViewController: UIViewController {
      override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
          resetValues()
          isMeasuring = true
          targetImageView.image = UIImage(named: "targetGreen")
      }
  
      override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
          isMeasuring = false
          targetImageView.image = UIImage(named: "targetWhite")
          if let line = currentLine {
              lines.append(line)
              currentLine = nil
              resetButton.isHidden = false
              resetImageView.isHidden = false
          }
      }
  }
  ```
    
- compute distance
  ```swift
  extension ViewController: ARSCNViewDelegate {
      func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
          DispatchQueue.main.async { [weak self] in
              self?.detectObjects()
          }
      }
  }
  
  extension ViewController {
      fileprivate func detectObjects() {
          guard let worldPosition =
            sceneView.realWorldVector(screenPosition: view.center) else { return }
          targetImageView.isHidden = false
          meterImageView.isHidden = false
          if lines.isEmpty {
              messageLabel.text = "Hold screen & move your phone…"
          }
          loadingView.stopAnimating()
          if isMeasuring {
              if startValue == vectorZero {
                  startValue = worldPosition
                  currentLine = Line(sceneView: sceneView, startVector: startValue, unit: unit)
              }
              endValue = worldPosition
              currentLine?.update(to: endValue)
              messageLabel.text = currentLine?.distance(to: endValue) ?? "Calculating…"
          }
      }
  }
  
  extension ARSCNView {
      func realWorldVector(screenPosition: CGPoint) -> SCNVector3? {
          let results = self.hitTest(screenPosition, types: [.featurePoint])
          guard let result = results.first else { return nil }
          return SCNVector3.positionFromTransform(result.worldTransform)
      }
  }
  
  final class Line {
      func distance(to vector: SCNVector3) -> String {
          return String(format: "%.2f%@", startVector.distance(from: vector) * unit.fator, unit.unit)
      }
  }
  ```
