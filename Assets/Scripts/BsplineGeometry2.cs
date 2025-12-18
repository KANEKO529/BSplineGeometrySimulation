using Unity.Mathematics;
using UnityEngine;
using System.IO;
using System.Text;
using System.Globalization;
using System;

using Unity.Jobs;
using Unity.Collections;


public class BsplineGeometry2
{
    //　制御点データ
    float2[] ctrl;
    // ノットベクトル
    float[] knots;
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
    int Ns;
    public float ds;
    private int k;  
    private int p;  
    private int n;   
    private int m;  
    public float[] u;
    private float u_min;
    private float u_max;

    private float d1Rx1du1, d1Ry1du1;
    private float d2Rx1du2, d2Ry1du2;
    private float d3Rx1du3, d3Ry1du3;
    private float d4Rx1du4, d4Ry1du4;

    private float[,] Nmat;

    public void Initialize()
    {

        // ① CSV読み込みが最初
        // ctrl = new NativeArray<float2>(ReadCSV.ReadControlPoints(), Allocator.Persistent);
        ctrl = ReadCSV.ReadControlPoints();
        Nctrl = ctrl.Length;

        n = Nctrl - 1;

        // ② Nctrl を使って n, m を決定
        k = 6; 
        p = k - 1; //次数 
        // N = 120001;
        Ns = 200001;

        // Ns x 2
        points = new float2[Ns];

        Debug.Log("BsplineGeometry Initialize 完了");
    }


    // function [N_matrix, x] = myfunc_calBsplineBasisMat(k, tValue, knot)
    // % 基底関数の行列を計算する関数
    // %【入力】
    // % k:Bスプラインの次数+1
    // % tValue:評価点のベクトル
    // % knot:ノットベクトル（非減少のベクトル）
    // %【出力】
    // % N_matrix - 基底関数の行列（サイズ：[length(x) x n]）

    //     p = k-1;
    //     x = tValue;

    //     % 基底関数の数
    //     n = length(knot)-p-1;
        
    //     % 基底関数の行列を初期化
    //     N_matrix = zeros(length(x), n);

    //     % 各評価点に対して基底関数を計算
    //     for j = 1:length(x)
    //         N = zeros(1, n);
    //         for i = 1:n
    //             N(i) = myfunc_bsplineBasis(i, p, knot, x(j));
    //         end
    //         N_matrix(j, :) = N;
    //     end
    // end

    // 基底関数の行列を計算する関数
    // 【Input】
    // k:Bスプラインの次数+1
    // tValue:評価点のベクトル
    // knots:ノットベクトル（非減少のベクトル）
    // 【Output】
    // N_matrix - 基底関数の行列（サイズ：[Ns x n]）
    private float[,] CalculateBsplineBasisMatrix(
        int k, 
        float[] tValue, 
        float[] knots
    )
    {

        int p = k - 1;
        int n = knots.Length - p - 1;

        // 基底関数の数
        int Nt = tValue.Length;

        // 基底関数の行列を初期化
        float[,] Nmat = new float[Nt, n];

        for (int j = 0; j < Nt; j++)
        {
            float x = tValue[j];
            for (int i = 0; i < n; i++)
            {
                Nmat[j, i] = BsplineBasis(i, p, knots, x);
            }
        }
        return Nmat;
    }


    // Bスプラインの基底関数 N_{i,p}(x) を計算する関数
    // 入力:
    // i - 基底関数のインデックス
    // p - Bスプラインの次数
    // t - ノットベクトル（非減少のベクトル）
    // x - 評価する点
    // 出力:
    // N - 基底関数の値 N_{i,p}(x)
    static float BsplineBasis(int i, int p, float[] knots, float x)
    {
        if (p == 0)
        {
            // MATLAB: if k(end-1) > x
            if (knots[knots.Length - 2] > x)
            {
                if(knots[i] <= x && x < knots[i + 1])
                    return 1f;
                else
                    return 0f;
            }
            else
            {
                if(knots[i] <= x && x <= knots[i + 1])
                    return 1f;
                else
                    return 0f;
            }
        }
        else
        {
            float denom1 = knots[i + p] - knots[i];
            float denom2 = knots[i + p + 1] - knots[i + 1];

            float term1, term2;

            if(denom1 == 0f)
                term1 = 0f;
            else
                term1 = (x - knots[i]) / denom1 * BsplineBasis(i, p - 1, knots, x);

            if(denom2 == 0f)
                term2 = 0f;
            else
                term2 = (knots[i + p + 1] - x) / denom2 * BsplineBasis(i + 1, p - 1, knots, x);

            return term1 + term2;
        }
    }

   
    //     function N = myfunc_bsplineBasis(i, p, k, x)
    // % Bスプラインの基底関数 N_{i,p}(x) を計算する関数
    // % 入力:
    // %   i - 基底関数のインデックス
    // %   p - Bスプラインの次数
    // %   t - ノットベクトル（非減少のベクトル）
    // %   x - 評価する点
    // %
    // % 出力:
    // %   N - 基底関数の値 N_{i,p}(x)

    //    if p == 0
    //        % 次数0の基底関数
    //        if k(end-1) > x
    //            if k(i) <= x && x < k(i+1)
    //                N = 1; 
    //            else
    //                N = 0;
    //            end
    //        else
    //            if k(i) <= x && x <= k(i+1)
    //                N = 1; 
    //            else
    //                N = 0;
    //            end
    //        end
    //    else
    //        % 次数pの基底関数を再帰的に計算
    //        denom1 = k(i+p) - k(i);
    //        denom2 = k(i+p+1) - k(i+1);
    //    
    //        % ゼロ割りを避ける
    //        if denom1 == 0
    //            term1 = 0;
    //        else
    //            term1 = (x - k(i)) / denom1 * myfunc_bsplineBasis(i, p-1, k, x);
    //        end
    //    
    //        if denom2 == 0
    //            term2 = 0;
    //        else
    //            term2 = (k(i+p+1) - x) / denom2 * myfunc_bsplineBasis(i+1, p-1, k, x);
    //        end
    //    
    //        N = term1 + term2;
    //    end
    // end


    static float2[] EvaluateBspline(
        float[,] Nmat,
        float2[] controlPoints)
    {
        int Nt = Nmat.GetLength(0);
        int n  = Nmat.GetLength(1);

        float2[] C = new float2[Nt];

        for (int j = 0; j < Nt; j++)
        {
            float2 sum = float2.zero;
            for (int i = 0; i < n; i++)
            {
                sum += Nmat[j, i] * controlPoints[i];
            }
            C[j] = sum;
        }
        return C;
    }



    public void CalculationBsplineGeometry()
    {

        // t_ver = 0:9
        float[] t_ver = new float[Nctrl];
        for (int i = 0; i < Nctrl; i++)
            t_ver[i] = i;

        // ノットベクトルの算出
        knots = GenerateKnots(Nctrl, p, t_ver);

        // 曲線を細かくサンプリングするパラメータの生成
        float[] t_sampling = new float[Ns];
        for (int i = 0; i < Ns; i++)
            t_sampling[i] = math.lerp(t_ver[0], t_ver[^1], (float)i / (Ns - 1));
        // Bスプライン基底関数行列の算出
        // Ns x n行列
        Debug.Log("CalculateBsplineBasisMatrix(k, t_sampling, knots) Start");
        Nmat = CalculateBsplineBasisMatrix(k, t_sampling, knots);

        Debug.Log("EvaluateBspline(Nmat, ctrl) Start");
        points = EvaluateBspline(Nmat, ctrl);
    }

    // maxKnots = (N_sampling-1)+p+1;
    // numKnotsRoom = maxKnots-2*p-1;
    // endVal = t_ver(end);
    // knot_start = zeros(1+p,1);
    // knot_end = endVal*ones(1+p,1);
    // knot_middle = zeros(numKnotsRoom,1);

    // for j = 1:numKnotsRoom
    //     j_start = j+1;
    //     j_end = j+p-1+1;
    //     knot_middle(j) = sum(t_ver(j_start:j_end))/p;
    // end
    // knot = [knot_start;knot_middle;knot_end];


    // ノットベクトルを計算する関数
    // 入力:
    // N_sampling - 基底関数のインデックス
    // p - Bスプラインの次数
    // t_ver - ノットベクトル（非減少のベクトル）
    // 出力:
    // knots 
    private float[] GenerateKnots(
        int N_sampling,
        int p,
        float[] t_ver
    )
    {
        int maxKnots = (N_sampling - 1) + p + 1;
        int numKnotsRoom = maxKnots - 2 * p - 1;

        float endVal = t_ver[t_ver.Length - 1];

        // knot_start, knot_end
        float[] knot = new float[maxKnots + 1];

        // start
        // 最初のp+1個
        for (int i = 0; i <= p; i++)
            knot[i] = 0f;

        // middle
        for (int j = 0; j < numKnotsRoom; j++)
        {
            int j_start = j + 1;
            int j_end   = j + p;

            float sum = 0f;
            for (int i = j_start; i <= j_end; i++)
                sum += t_ver[i];

            knot[p + 1 + j] = sum / p;
        }

        // end
        // 最後ののp+1個
        for (int i = maxKnots - p; i <= maxKnots; i++)
            knot[i] = endVal;

        return knot;
    }
}
