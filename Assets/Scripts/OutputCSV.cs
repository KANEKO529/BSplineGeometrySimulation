using UnityEngine;
using Unity.Mathematics;
using System.IO;
using System.Text;
using System.Globalization;

public static class OutputCSV
{
    public static void OutputBsplineGeometryData(
        float2[] points,
        float2[] derivative1,
        float2[] derivative2,
        float2[] derivative3,
        float2[] derivative4,
        float ds)
    {
        // 日付文字列 (例: 2025-11-29_18-10-12)
        string time = System.DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        string fileName = $"Bspline_output_{time}.csv";

        using (var writer = new StreamWriter(fileName, false, Encoding.GetEncoding("shift_jis")))
        {
            // ヘッダー
            writer.WriteLine(
                "s," +
                "Rx1(u),Ry1(u)," +
                "d1Rx1du1,d1Ry1du1," +
                "d2Rx1du2,d2Ry1du2," +
                "d3Rx1dy3,d3Ry1du3" +
                "d4Rx1dy4,d4Ry1du4"
            );

            int n = points.Length;

            for (int i = 0; i < n; i++)
            {
                float s = i * ds;

                writer.WriteLine(
                    $"{F(s)}," +
                    $"{F(points[i].x)},{F(points[i].y)}," +
                    $"{F(derivative1[i].x)},{F(derivative1[i].y)}," +
                    $"{F(derivative2[i].x)},{F(derivative2[i].y)}," +
                    $"{F(derivative3[i].x)},{F(derivative3[i].y)}," +
                    $"{F(derivative4[i].x)},{F(derivative4[i].y)}"
                );
            }
        }

        Debug.Log($"CSV 出力完了: {fileName}");
    }

    // ---- NaN の時は "" を返す ----
    static string F(float value)
    {
        return float.IsNaN(value) ? "" : value.ToString("F6", CultureInfo.InvariantCulture);
    }
}
