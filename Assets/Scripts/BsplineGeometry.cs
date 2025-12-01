using Unity.Mathematics;
using UnityEngine;
using System.IO;
using System.Text;
using System.Globalization;
using System;


public class BsplineGeometry
{

    public SimulationManager sim; 

    //　制御点データ
    float2[] ctrl;
    //　制御点データ　 1階微分
    float2[] beta1;
    //　制御点データ 1階微分
    float2[] beta2;
    //　制御点データ 1階微分
    float2[] beta3;
    //　制御点データ 1階微分
    float2[] beta4;
    // ノットベクトル
    private float[] knots, knots1, knots2, knots3, knots4;
    //　Bスプライン曲線 R(u)
    public float2[] points;
    // 1階微分　dR(u)/du
    public float2[] derivative1;
    // 2階微分
    public float2[] derivative2;
    // 3階微分
    public float2[] derivative3;
    // 4階微分
    public float2[] derivative4;

    int Nctrl; // 制御点の総数　1202
    int N;// 120100
    float ds;
    private int k;   // 次数（3）
    private int n;   // 制御点数−1
    private float u_min;
    private float u_max;

    private float d1Rx1du1, d1Ry1du1;
    private float d2Rx1du2, d2Ry1du2;
    private float d3Rx1du3, d3Ry1du3;
    private float d4Rx1du4, d4Ry1du4;


    public void Initialize()
    {

        sim = new SimulationManager();

        // ① CSV読み込みが最初
        ctrl = ReadCSV.ReadControlPoints();
        Nctrl = ctrl.Length;  // ← 1201 or 1202 が入る

        // ② Nctrl を使って n, m を決定
        k = 5;               
        n = Nctrl - 1;      // ← 1201

        // ⑤ 出力配列を確保
        N = 120001;
        points      = new float2[N];
        derivative1 = new float2[N];
        derivative2 = new float2[N];
        derivative3 = new float2[N];
        derivative4 = new float2[N];

        ds = 0.0001f;

        knots = GenerateKnots(n, k);

        knots1 = SubKnots(knots, 1);
        knots2 = SubKnots(knots, 2);
        knots3 = SubKnots(knots, 3);
        knots4 = SubKnots(knots, 4);

        Debug.Log("Initialize 完了");
    }


    public void CalculationBsplineGeometry()
    {

        // 1〜4階まで制御点生成
        beta1 = DerivativeCtrl1(ctrl);
        beta2 = DerivativeCtrl2(beta1);
        beta3 = DerivativeCtrl3(beta2);
        beta4 = DerivativeCtrl4(beta3);

        float u0 = knots[k];
        float u1 = knots[n + 1];

        for (int i = 0; i < N; i++)
        {
            float t = (float)i / (N - 1);
            float u = math.lerp(u0, u1, t);

            points[i]      = CalculateBsplinePoint(u, ctrl, k, knots);

            derivative1[i] = CalculateBsplinePointDerivative(u, beta1, k-1, knots1);

            derivative2[i] = CalculateBsplinePointDerivative(u, beta2, k-2, knots2);

            derivative3[i] = CalculateBsplinePointDerivative(u, beta3, k-3, knots3);

            derivative4[i] = CalculateBsplinePointDerivative(u, beta4, k-4, knots4);

        }

        OutputCSV.OutputBsplineGeometryData(points, derivative1, derivative2, derivative3, derivative4, ds);

        sim.EndSimulation();
    }

    float[] SubKnots(float[] knots, int r)
    {
        int newLen = knots.Length - 2 * r;
        float[] newKnots = new float[newLen];
        Array.Copy(knots, r, newKnots, 0, newLen);
        return newKnots;
    }



    private float[] GenerateKnots(int n, int degree)
    {

        int m = n + degree + 1;

        float[] knots = new float[m + 1]; // 1206 要素

        // 開一様ノット
        for (int i = 0; i <= degree; i++)
            knots[i] = 0;

        for (int i = degree + 1; i <= n; i++)
            knots[i] = i - degree;

        for (int i = n + 1; i < m + 1; i++)
            knots[i] = n - degree + 1;

        return knots;

        // Debug.Log($"knots:{knots[0]}, {knots[1]}, {knots[2]}, {knots[3]}, {knots[4]}, ... , {knots[m-5]}, {knots[m-4]}, {knots[m-3]}, {knots[m-2]}, {knots[m-1]}");
    }


    // beta1
    float2[] DerivativeCtrl1(float2[] ctrl)
    {
        int k = 5;

        float2[] beta1 = new float2[ctrl.Length - 1];

        for (int i = 0; i < ctrl.Length - 1; i++)
        {
            float denom = knots[i + 1 + k] - knots[i + 1];
            float scale = (denom == 0) ? 0 : k / denom;

            beta1[i] = scale * (ctrl[i + 1] - ctrl[i]);
        }

        return beta1;
    }

    float2[] DerivativeCtrl2(float2[] ctrl)
    {
        int k = 4;

        float2[] beta2 = new float2[ctrl.Length - 1];

        for (int i = 0; i < ctrl.Length - 1; i++)
        {
            float denom = knots[i + 1 + k] - knots[i + 1];
            float scale = (denom == 0) ? 0 : k / denom;

            beta2[i] = scale * (ctrl[i + 1] - ctrl[i]);
        }

        return beta2;
    }

    float2[] DerivativeCtrl3(float2[] ctrl)
    {
        int k = 3;

        float2[] beta3 = new float2[ctrl.Length - 1];

        for (int i = 0; i < ctrl.Length - 1; i++)
        {
            float denom = knots[i + 1 + k] - knots[i + 1];
            float scale = (denom == 0) ? 0 : k / denom;

            beta3[i] = scale * (ctrl[i + 1] - ctrl[i]);
        }

        return beta3;
    }

    float2[] DerivativeCtrl4(float2[] ctrl)
    {
        int k = 2;

        float2[] beta4 = new float2[ctrl.Length - 1];

        for (int i = 0; i < ctrl.Length - 1; i++)
        {
            float denom = knots[i + 1 + k] - knots[i + 1];
            float scale = (denom == 0) ? 0 : k / denom;

            beta4[i] = scale * (ctrl[i + 1] - ctrl[i]);
        }

        return beta4;
    }

    private int FindSpan(float u, int degree, int n, float[] knots)
    {
        if (u >= knots[n + 1])
            return n;

        int low = degree;
        int high = n + 1;
        int mid = (low + high) / 2;

        while (u < knots[mid] || u >= knots[mid + 1])
        {
            if (u < knots[mid]) high = mid;
            else low = mid;
            mid = (low + high) / 2;
        }

        return mid;
    }

    public float2 CalculateBsplinePoint(float u, float2[] ctrl, int degree, float[] knots)
    {
        return DeBoorEvaluate(ctrl, u, degree, knots);
    }

    private float2 CalculateBsplinePointDerivative(float u, float2[] ctrl, int degree, float[] knots)
    {
        return DeBoorEvaluate(ctrl, u, degree, knots);
    }

    private float2 DeBoorEvaluate(float2[] ctrl, float u, int degree, float[] knots)
    {
        int n = ctrl.Length - 1;

        // int span = FindSpan(u, degree);
        int span = FindSpan(u, degree, ctrl.Length - 1, knots);


        float2[] d = new float2[degree + 1];

        for (int j = 0; j <= degree; j++)
        {
            d[j] = ctrl[span - degree + j];
        }

        for (int r = 1; r <= degree; r++)
        {
            for (int j = degree; j >= r; j--)
            {
                int i = span - degree + j;
                float denom = knots[i + degree - r + 1] - knots[i];
                float alpha = (denom == 0) ? 0 : (u - knots[i]) / denom;

                d[j] = math.lerp(d[j - 1], d[j], alpha);
            }
        }

        return d[degree];
    }

    private float Alpha(float u, int i, int r)
    {
        float denom = knots[i + k - r + 1] - knots[i];
        return (denom == 0) ? 0 : (u - knots[i]) / denom;
    }


}
