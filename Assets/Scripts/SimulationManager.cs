using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using System.IO;
using System;
using System.Text;            // UTF8Encoding ← これが不足でCS0246

public class SimulationManager : MonoBehaviour {

    [Header("シミュレーション制御")]
    public bool isSimulationRunning = false;  // シミュレーション実行状態

    public BsplineGeometry bsplineGeometry;    

    void Start()
    {
        bsplineGeometry = new BsplineGeometry();

        // ここで初期化
        bsplineGeometry.Initialize();
    }

    void Update() {
        // キーボード入力処理
        HandleInput();
    }
    
    private void HandleInput()
    {
        // Sキーでシミュレーション開始 or 再スタート
        if (Input.GetKeyDown(KeyCode.S))
        {
            StartSimulation();
        }
    }

    public void StartSimulation()
    {
        //  一番最初のシミュレーション開始
        isSimulationRunning = true;
        Debug.Log("シミュレーション開始");

        bsplineGeometry.CalculationBsplineGeometry();
    }

    public void EndSimulation()
    {
        isSimulationRunning = false;
        Debug.Log("シミュレーション終了");
        return;
    }
}
