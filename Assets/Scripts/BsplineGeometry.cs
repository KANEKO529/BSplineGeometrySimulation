using Unity.Mathematics;
using UnityEngine;
using System.IO;
using System.Text;
using System.Globalization;
using System;

using Unity.Jobs;
using Unity.Collections;


public class BsplineGeometry
{
    //　制御点データ
    NativeArray<float2> ctrl;
    //　制御点データ　 1階微分
    NativeArray<float2> beta1;
    //　制御点データ 1階微分
    NativeArray<float2> beta2;
    //　制御点データ 1階微分
    NativeArray<float2> beta3;
    //　制御点データ 1階微分
    NativeArray<float2> beta4;
    // ノットベクトル
    NativeArray<float> knots, knots1, knots2, knots3, knots4;
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
    public float ds;
    private int k;   // 次数（3）
    private int n;   // 制御点数−1
    public float[] u;
    private float u_min;
    private float u_max;

    private float d1Rx1du1, d1Ry1du1;
    private float d2Rx1du2, d2Ry1du2;
    private float d3Rx1du3, d3Ry1du3;
    private float d4Rx1du4, d4Ry1du4;

    public void Initialize()
    {

        // ① CSV読み込みが最初
        ctrl = new NativeArray<float2>(ReadCSV.ReadControlPoints(), Allocator.Persistent);
        Nctrl = ctrl.Length;  // ← 1201 or 1202 が入る

        // ② Nctrl を使って n, m を決定
        k = 5;               
        n = Nctrl - 1;      // ← 1201

        // ⑤ 出力配列を確保
        N = 120001;
        u = new float[N];
        points      = new float2[N];
        derivative1 = new float2[N];
        derivative2 = new float2[N];
        derivative3 = new float2[N];
        derivative4 = new float2[N];

        beta1 = new NativeArray<float2>(n, Allocator.TempJob);
        beta2 = new NativeArray<float2>(n - 1, Allocator.TempJob);
        beta3 = new NativeArray<float2>(n - 2, Allocator.TempJob);
        beta4 = new NativeArray<float2>(n - 3, Allocator.TempJob);

        ds = 0.0001f;

        knots = GenerateKnots(n, k);

        knots1 = SubKnots(knots, 1);
        knots2 = SubKnots(knots, 2);
        knots3 = SubKnots(knots, 3);
        knots4 = SubKnots(knots, 4);

        Debug.Log("BsplineGeometry Initialize 完了");
    }


    public void CalculationBsplineGeometry()
    {

        CalculateDerivativeControlPointsJob();

        u_min = knots[k];
        u_max = knots[n + 1];

        for (int i = 0; i < N; i++)
        {
            float t = (float)i / (N - 1);
            float u_local = math.lerp(u_min, u_max, t);

            u[i] = u_local;

            points[i]      = CalculateBsplinePoint(u_local, ctrl, k, knots);

            EvaluateDerivativesAtU(u_local, i);
        }
    }

    NativeArray<float> SubKnots(NativeArray<float> knots, int r)
    {
        int newLen = knots.Length - 2 * r;
        var newKnots = new NativeArray<float>(newLen, Allocator.Persistent);

        for (int i = 0; i < newLen; i++)
            newKnots[i] = knots[i + r];

        return newKnots;
    }

    private NativeArray<float> GenerateKnots(int n, int degree)
    {

        int m = n + degree + 1;
        var knots = new NativeArray<float>(m + 1, Allocator.Persistent);
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

    private int FindSpan(float u, int degree, int n, NativeArray<float> knots)
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

    public float2 CalculateBsplinePoint(float u, NativeArray<float2> ctrl, int degree, NativeArray<float> knots)
    {
        return DeBoorEvaluate(ctrl, u, degree, knots);
    }

    private float2 DeBoorEvaluate(NativeArray<float2> ctrl, float u, int degree, NativeArray<float> knots)
    {
        int n = ctrl.Length - 1;

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

    public float GetUMin()
    {
        return u_min;
    }

    public float GetUMax()
    {
        return u_max;
    }

    public void CalculateDerivativeControlPointsJob()
    {
        int N1 = ctrl.Length - 1;

        var j1 = new DerivativeCtrlJob { 
            ctrl = ctrl, 
            knots = knots, 
            degree = 5, 
            beta = beta1 }.Schedule(N1, 64);

        var j2 = new DerivativeCtrlJob { 
            ctrl = beta1, 
            knots = knots, 
            degree = 4, 
            beta = beta2 }.Schedule(N1 - 1, 64, j1);

        var j3 = new DerivativeCtrlJob { 
            ctrl = beta2, 
            knots = knots, 
            degree = 3, 
            beta = beta3 }.Schedule(N1 - 2, 64, j2);

        var j4 = new DerivativeCtrlJob { 
            ctrl = beta3, 
            knots = knots, 
            degree = 2, 
            beta = beta4 }.Schedule(N1 - 3, 64, j3);

        j4.Complete();
    }

    public void EvaluateDerivativesAtU(float u_local, int idx)
    {
        var res1 = new NativeArray<float2>(1, Allocator.TempJob);
        var res2 = new NativeArray<float2>(1, Allocator.TempJob);
        var res3 = new NativeArray<float2>(1, Allocator.TempJob);
        var res4 = new NativeArray<float2>(1, Allocator.TempJob);

        var j1 = new BSplineDerivativeJob {
            ctrl = beta1, 
            knots = knots1, 
            u = u_local, 
            degree = k - 1, 
            result = res1
        }.Schedule();

        var j2 = new BSplineDerivativeJob {
            ctrl = beta2, 
            knots = knots2, 
            u = u_local, 
            degree = k - 2, 
            result = res2
        }.Schedule();

        var j3 = new BSplineDerivativeJob {
            ctrl = beta3, 
            knots = knots3, 
            u = u_local, 
            degree = k - 3, 
            result = res3
        }.Schedule();

        var j4 = new BSplineDerivativeJob {
            ctrl = beta4, 
            knots = knots4, 
            u = u_local, 
            degree = k - 4, 
            result = res4
        }.Schedule();

        // 全部待機
        var h123 = JobHandle.CombineDependencies(j1, j2, j3);
        var hAll = JobHandle.CombineDependencies(h123, j4);

        hAll.Complete();

        derivative1[idx] = res1[0];
        derivative2[idx] = res2[0];
        derivative3[idx] = res3[0];
        derivative4[idx] = res4[0];
    }

    void OnDestroy()
    {
        if (ctrl.IsCreated) ctrl.Dispose();

        if (beta1.IsCreated) beta1.Dispose();
        if (beta2.IsCreated) beta1.Dispose();
        if (beta3.IsCreated) beta1.Dispose();
        if (beta4.IsCreated) beta1.Dispose();

        if (knots1.IsCreated) knots1.Dispose();
        if (knots2.IsCreated) knots2.Dispose();
        if (knots3.IsCreated) knots3.Dispose();
        if (knots4.IsCreated) knots4.Dispose();
    }
}

public struct DerivativeCtrlJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float2> ctrl;
    [ReadOnly] public NativeArray<float> knots;
    public int degree;

    [WriteOnly] public NativeArray<float2> beta;

    public void Execute(int i)
    {
        float denom = knots[i + 1 + degree] - knots[i + 1];
        float scale = (denom == 0) ? 0 : degree / denom;

        beta[i] = scale * (ctrl[i + 1] - ctrl[i]);
    }
}


public struct BSplineDerivativeJob : IJob
{
    [ReadOnly] public NativeArray<float2> ctrl;
    [ReadOnly] public NativeArray<float> knots;
    public float u;
    public int degree;

    [WriteOnly] public NativeArray<float2> result;

    public void Execute()
    {
        result[0] = DeBoor(ctrl, u, degree, knots);
    }

    private float2 DeBoor(NativeArray<float2> c, float u, int k, NativeArray<float> K)
    {
        int n = c.Length - 1;
        int span = FindSpan(u, k, n, K);

        float2[] d = new float2[k + 1];
        for (int j = 0; j <= k; j++)
            d[j] = c[span - k + j];

        for (int r = 1; r <= k; r++)
        {
            for (int j = k; j >= r; j--)
            {
                int idx = span - k + j;
                float denom = K[idx + k - r + 1] - K[idx];
                float alpha = (denom == 0) ? 0 : (u - K[idx]) / denom;
                d[j] = math.lerp(d[j - 1], d[j], alpha);
            }
        }
        return d[k];
    }

    private int FindSpan(float u, int degree, int n, NativeArray<float> knots)
    {
        if (u >= knots[n + 1]) return n;

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
}