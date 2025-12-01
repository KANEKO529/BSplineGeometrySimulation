using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PathKinematics
{

    public BsplineGeometry bsplineGeometry;  // ← 外からセット

    public float[] cs1;
    public float[] d1c1ds11;
    public float[] d2c1ds12;


    public float _rx1;
    public float _ry1;

    private float _d1rx1du11, _d1ry1du11;
    private float _d2rx1du12, _d2ry1du12;
    private float _d3rx1du13, _d3ry1du13;
    private float _d4rx1du14, _d4ry1du14;

    private float _squaredRx1;
    private float _squaredRy1;

    private float _squaredD1rx1du11;
    private float _cubedD1rx1du11;
    private float _squaredD1ry1du11;
    private float _cubedD1ry1du11;

    private float _squaredD2rx1du12;
    private float _cubedD2rx1du12;
    private float _squaredD2ry1du12;
    private float _cubedD2ry1du12;

    private float _formulaOfD1ry1du11MulD2rx1du12;
    private float _formulaOfD1ry1du11MulD3rx1du13;
    private float _formulaOfD1ry1du11MulD3ry1du13;
    private float _formulaOfD1rx1du11MulD2ry1du12;
    private float _formulaOfD2rx1du12MulD2ry1du12;

    private float _formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11;

    public void Initialize(BsplineGeometry geo)
    {
        bsplineGeometry = geo;

        int N = bsplineGeometry.points.Length;
        cs1 = new float[N];
        d1c1ds11 = new float[N];
        d2c1ds12 = new float[N];


        Debug.Log("PathKinematics Initialize 完了");
    }
    public void CalculationPathKinematics()
    {
        int N = 120001;
        
        for (int i = 0; i < N; i++)
        {
            ComputeUtilitiesForBezierGeometry(i);
            // 曲率計算
            cs1[i] = CalculateCurvatureOfU1(); //ok
            d1c1ds11[i] = CalculateCurvatureOfU1Derivative1(); // ok
            d2c1ds12[i] = CalculateCurvatureOfU1Derivative2();
        }
    }


    // 曲率計算 ok
    public float CalculateCurvatureOfU1()
    {
        float cs1;

        cs1 = -((_squaredD1ry1du11*(_formulaOfD1ry1du11MulD2rx1du12 - _formulaOfD1rx1du11MulD2ry1du12)) / Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 2.5f))
        + (_squaredD1rx1du11*(-(_formulaOfD1ry1du11MulD2rx1du12) + _formulaOfD1rx1du11MulD2ry1du12)) / Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 2.5f);

        return cs1;
    }
    
    // 曲率1次微分 ok
    public float CalculateCurvatureOfU1Derivative1()
    {
        float d1c1ds11;

        d1c1ds11 = (_d1rx1du11*((_d1ry1du11*Mathf.Pow(-(_formulaOfD1ry1du11MulD2rx1du12) + _formulaOfD1rx1du11MulD2ry1du12,2)) /
                Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 3.5f) + (-(_cubedD1ry1du11*_squaredD2rx1du12) + _d1rx1du11*_squaredD1ry1du11 *
                    (5.0f*_formulaOfD2rx1du12MulD2ry1du12 - _formulaOfD1ry1du11MulD3rx1du13) - _cubedD1rx1du11*(3.0f*_formulaOfD2rx1du12MulD2ry1du12 + _formulaOfD1ry1du11MulD3rx1du13) +
                    Mathf.Pow(_d1rx1du11, 4)*_d3ry1du13 + _squaredD1rx1du11*_d1ry1du11*(3.0f*_squaredD2rx1du12 - 4.0f*_squaredD2ry1du12 + _formulaOfD1ry1du11MulD3ry1du13)) /
                Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 3.5f))) / Mathf.Sqrt(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11) -
                (_d1ry1du11*((_d1rx1du11 *Mathf.Pow(-(_formulaOfD1ry1du11MulD2rx1du12) +_formulaOfD1rx1du11MulD2ry1du12,2)) /
                    Mathf.Pow(_squaredD1rx1du11 +_squaredD1ry1du11, 3.5f) + (_cubedD1ry1du11*(-3*_formulaOfD2rx1du12MulD2ry1du12 + _formulaOfD1ry1du11MulD3rx1du13) +
                        _squaredD1rx1du11*_d1ry1du11*(5.0f*_formulaOfD2rx1du12MulD2ry1du12 + _formulaOfD1ry1du11MulD3rx1du13) -
                        _d1rx1du11*_squaredD1ry1du11*(4.0f*_squaredD2rx1du12 -
                            3.0f*_squaredD2ry1du12 + _formulaOfD1ry1du11MulD3ry1du13) - _cubedD1rx1du11*(_squaredD2ry1du12 + _formulaOfD1ry1du11MulD3ry1du13)) /
                    Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 3.5f))) / Mathf.Sqrt(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11);


        return d1c1ds11;
    }
    
    // 曲率2次微分 ok
    public float CalculateCurvatureOfU1Derivative2()
    {

        float _d2c1ds12;

        _d2c1ds12 = (-(Mathf.Pow(_d1rx1du11, 4) *
                (4.0f*_d2ry1du12*_d3rx1du13 +
                    6.0f*_d2rx1du12*_d3ry1du13 +
                    _d1ry1du11*_d4rx1du14)) +
                _squaredD1rx1du11*_d1ry1du11 *
                (-15.0f*_cubedD2rx1du12 +
                    _d2rx1du12*(39.0f*_squaredD2ry1du12 -
                        2.0f*_formulaOfD1ry1du11MulD3ry1du13) +
                    2.0f*_d1ry1du11*(_d2ry1du12*_d3rx1du13 -
                        _d1ry1du11*_d4rx1du14)) +
                _cubedD1ry1du11 *
                (3.0f*_cubedD2rx1du12 +
                    _d2rx1du12*(-15.0f*_squaredD2ry1du12 +
                        4.0f*_formulaOfD1ry1du11MulD3ry1du13) +
                    _d1ry1du11*(6.0f*_d2ry1du12*_d3rx1du13 -
                        _d1ry1du11*_d4rx1du14)) +
                Mathf.Pow(_d1rx1du11, 5)*_d4ry1du14 +
                _d1rx1du11*_squaredD1ry1du11 *
                (-39.0f*_squaredD2rx1du12*_d2ry1du12 +
                    15.0f*_cubedD2ry1du12 +
                    10.0f*_formulaOfD1ry1du11MulD2rx1du12*_d3rx1du13 -
                    10.0f*_d1ry1du11*_d2ry1du12*_d3ry1du13 +
                    _squaredD1ry1du11*_d4ry1du14) +
                _cubedD1rx1du11 *
                (15.0f*_squaredD2rx1du12*_d2ry1du12 -
                    3.0f*_cubedD2ry1du12 +
                    10.0f*_formulaOfD1ry1du11MulD2rx1du12*_d3rx1du13 -
                    10.0f*_d1ry1du11*_d2ry1du12*_d3ry1du13 +
                    2.0f*_squaredD1ry1du11*_d4ry1du14)) /
                Mathf.Pow(_formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11, 4.5f);

        return _d2c1ds12; 
    }

    public void ComputeUtilitiesForBezierGeometry(int idx)
    {
        _rx1 = bsplineGeometry.points[idx].x;
        _ry1 = bsplineGeometry.points[idx].y;

        _d1rx1du11 = bsplineGeometry.derivative1[idx].x;
        _d1ry1du11 = bsplineGeometry.derivative1[idx].y;
        _d2rx1du12 = bsplineGeometry.derivative2[idx].x;
        _d2ry1du12 = bsplineGeometry.derivative2[idx].y;
        _d3rx1du13 = bsplineGeometry.derivative3[idx].x;
        _d3ry1du13 = bsplineGeometry.derivative3[idx].y;
        _d4rx1du14 = bsplineGeometry.derivative4[idx].x;
        _d4ry1du14 = bsplineGeometry.derivative4[idx].y;

        _squaredRx1 = Mathf.Pow(_rx1,2);
        _squaredRy1 = Mathf.Pow(_ry1,2);

        _squaredD1rx1du11 = Mathf.Pow(_d1rx1du11,2);
        _cubedD1rx1du11 = Mathf.Pow(_d1rx1du11,3);
        _squaredD1ry1du11 = Mathf.Pow(_d1ry1du11,2);
        _cubedD1ry1du11 = Mathf.Pow(_d1ry1du11,3);

        _squaredD2rx1du12 = Mathf.Pow(_d2rx1du12,2);
        _cubedD2rx1du12 = Mathf.Pow(_d2rx1du12,3);
        _squaredD2ry1du12 = Mathf.Pow(_d2ry1du12,2);
        _cubedD2ry1du12 = Mathf.Pow(_d2ry1du12,3);

        _formulaOfD1ry1du11MulD2rx1du12 = _d1ry1du11*_d2rx1du12;
        _formulaOfD1ry1du11MulD3rx1du13 = _d1ry1du11*_d3rx1du13;
        _formulaOfD1ry1du11MulD3ry1du13 = _d1ry1du11*_d3ry1du13;
        _formulaOfD1rx1du11MulD2ry1du12 = _d1rx1du11*_d2ry1du12;
        _formulaOfD2rx1du12MulD2ry1du12 = _d2rx1du12*_d2ry1du12;

        _formulaOfSquaredD1rx1du11PlusSquaredD1ry1du11 = _squaredD1rx1du11 + _squaredD1ry1du11;
    }
}
