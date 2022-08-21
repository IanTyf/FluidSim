using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FluidSim : MonoBehaviour
{
    public int size;
    public int iteration;
    [Tooltip("A multiplier for dt. The larger the timestep is, the faster the fluid moves and the less accurate the calculation is, but may look nicer and sharper.")]
    public float timeStep;

    public Color[] colors;

    public Fluid fluid;

    // Start is called before the first frame update
    void Start()
    {
        // 3rd param is how fast the color fades: 0.000001 to 0.00001 looks good
        // 4th prarm is how fast the velocity fades (how 'sticky' the fluid is): 0.0000001 to 0.000001 looks good
        fluid = new Fluid(size, 0.05f, 0.000001f, 0.0000005f);

        colors = new Color[size * size];
    }

    // Update is called once per frame
    void Update()
    {
        fluid.dt = Time.deltaTime * timeStep;
        FluidCubeStep(fluid);
        updateColors(fluid);

        if (Input.GetKey(KeyCode.DownArrow))
        {
            fluid.AddDensity(size / 3, size / 2, 100f * Time.deltaTime, 0);
            fluid.AddVelocity(size / 3, size / 2, 0f, 100f * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            fluid.AddDensity(size / 3, size / 2, 100f * Time.deltaTime, 0);
            fluid.AddVelocity(size / 3, size / 2, -100f * Time.deltaTime, 0f);
        }
        if (Input.GetKey(KeyCode.UpArrow))
        {
            fluid.AddDensity(size / 3, size / 2, 100f * Time.deltaTime, 0);
            fluid.AddVelocity(size / 3, size / 2, 0f, -100f * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.LeftArrow))
        {
            fluid.AddDensity(size / 3, size / 2, 100f * Time.deltaTime, 0);
            fluid.AddVelocity(size / 3, size / 2, 100f * Time.deltaTime, 0f);
        }


        if (Input.GetKey(KeyCode.S))
        {
            fluid.AddDensity(2 * size / 3, size / 3, 100f * Time.deltaTime, 2);
            fluid.AddVelocity(2 * size / 3, size / 3, 0f, 100f * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.D))
        {
            fluid.AddDensity(2 * size / 3, size / 3, 100f * Time.deltaTime, 2);
            fluid.AddVelocity(2 * size / 3, size / 3, -100f * Time.deltaTime, 0f);
        }
        if (Input.GetKey(KeyCode.W))
        {
            fluid.AddDensity(2 * size / 3, size / 3, 100f * Time.deltaTime, 2);
            fluid.AddVelocity(2 * size / 3, size / 3, 0f, -100f * Time.deltaTime);
        }
        if (Input.GetKey(KeyCode.A))
        {
            fluid.AddDensity(2 * size / 3, size / 3, 100f * Time.deltaTime, 2);
            fluid.AddVelocity(2 * size / 3, size / 3, 100f * Time.deltaTime, 0f);
        }
    }

    public void FluidCubeStep(Fluid cube)
    {
        int N = cube.size;
        float visc = cube.viscosity;
        float diff = cube.diffuse;
        float dt = cube.dt;
        float[] Vx = cube.curVx;
        float[] Vy = cube.curVy;
        float[] Vx0 = cube.oldVx;
        float[] Vy0 = cube.oldVy;

        float[] sR = cube.oldDensityR;
        float[] densityR = cube.curDensityR;
        float[] sG = cube.oldDensityG;
        float[] densityG = cube.curDensityG;
        float[] sB = cube.oldDensityB;
        float[] densityB = cube.curDensityB;


        diffuse(1, Vx0, Vx, visc, dt, iteration, N);
        diffuse(2, Vy0, Vy, visc, dt, iteration, N);

        project(Vx0, Vy0, Vx, Vy, iteration, N);

        advect(1, Vx, Vx0, Vx0, Vy0, dt, N);
        advect(2, Vy, Vy0, Vx0, Vy0, dt, N);

        project(Vx, Vy, Vx0, Vy0, iteration, N);

        diffuse(0, sR, densityR, diff, dt, iteration, N);
        advect(0, densityR, sR, Vx, Vy, dt, N);

        diffuse(0, sG, densityG, diff, dt, iteration, N);
        advect(0, densityG, sG, Vx, Vy, dt, N);

        diffuse(0, sB, densityB, diff, dt, iteration, N);
        advect(0, densityB, sB, Vx, Vy, dt, N);
    }

    public int IX(int x, int y)
    {
        return x + y * size;
    }

    public void set_bnd(int b, float[] x, int N)
    {

        for (int i = 1; i < N - 1; i++)
        {
            x[IX(i, 0)] = b == 2 ? -x[IX(i, 1)] : x[IX(i, 1)];
            x[IX(i, N - 1)] = b == 2 ? -x[IX(i, N - 2)] : x[IX(i, N - 2)];
        }

        for (int j = 1; j < N - 1; j++)
        {
            x[IX(0, j)] = b == 1 ? -x[IX(1, j)] : x[IX(1, j)];
            x[IX(N - 1, j)] = b == 1 ? -x[IX(N - 2, j)] : x[IX(N - 2, j)];
        }

        x[IX(0, 0)] = 0.5f * (x[IX(1, 0)]
                                      + x[IX(0, 1)]);
        x[IX(0, N - 1)] = 0.5f * (x[IX(1, N - 1)]
                                      + x[IX(0, N - 2)]);
        x[IX(N - 1, 0)] = 0.5f * (x[IX(N - 2, 0)]
                                      + x[IX(N - 1, 1)]);
        x[IX(N - 1, N - 1)] = 0.5f * (x[IX(N - 2, N - 1)]
                                      + x[IX(N - 1, N - 2)]);
    }


    public void lin_solve(int b, float[] x, float[] x0, float a, float c, int iter, int N)
    {
        float cRecip = 1.0f / c;
        for (int k = 0; k < iter; k++)
        {
            for (int j = 1; j < N - 1; j++)
            {
                for (int i = 1; i < N - 1; i++)
                {
                    x[IX(i, j)] = (x0[IX(i, j)]
                                    + a * (x[IX(i + 1, j)]
                                        + x[IX(i - 1, j)]
                                        + x[IX(i, j + 1)]
                                        + x[IX(i, j - 1)])
                                  ) * cRecip;
                }
            }
            set_bnd(b, x, N);
        }
    }

    public void diffuse(int b, float[] x, float[] x0, float diff, float dt, int iter, int N)
    {
        float a = dt * diff * (N - 2) * (N - 2);
        lin_solve(b, x, x0, a, 1 + 6 * a, iter, N);
    }

    public void project(float[] velocX, float[] velocY, float[] p, float[] div, int iter, int N)
    {
        for (int j = 1; j < N - 1; j++)
        {
            for (int i = 1; i < N - 1; i++)
            {
                div[IX(i, j)] = -0.5f * (
                         velocX[IX(i + 1, j)]
                        - velocX[IX(i - 1, j)]
                        + velocY[IX(i, j + 1)]
                        - velocY[IX(i, j - 1)]
                    ) / N;
                p[IX(i, j)] = 0;
            }
        }
        set_bnd(0, div, N);
        set_bnd(0, p, N);
        lin_solve(0, p, div, 1, 6, iter, N);


        for (int j = 1; j < N - 1; j++)
        {
            for (int i = 1; i < N - 1; i++)
            {
                velocX[IX(i, j)] -= 0.5f * (p[IX(i + 1, j)]
                                                - p[IX(i - 1, j)]) * N;
                velocY[IX(i, j)] -= 0.5f * (p[IX(i, j + 1)]
                                                - p[IX(i, j - 1)]) * N;
            }
        }

        set_bnd(1, velocX, N);
        set_bnd(2, velocY, N);
    }

    public void advect(int b, float[] d, float[] d0, float[] velocX, float[] velocY, float dt, int N)
    {
        float i0, i1, j0, j1;

        float dtx = dt * (N - 2);
        float dty = dt * (N - 2);

        float s0, s1, t0, t1;
        float tmp1, tmp2, x, y;

        float Nfloat = N;
        float ifloat, jfloat;
        int i, j;

        for (j = 1, jfloat = 1; j < N - 1; j++, jfloat++)
        {
            for (i = 1, ifloat = 1; i < N - 1; i++, ifloat++)
            {
                tmp1 = dtx * velocX[IX(i, j)];
                tmp2 = dty * velocY[IX(i, j)];
                x = ifloat - tmp1;
                y = jfloat - tmp2;

                if (x < 0.5f) x = 0.5f;
                if (x > Nfloat + 0.5f) x = Nfloat + 0.5f;
                i0 = Mathf.Floor(x);
                i1 = i0 + 1.0f;
                if (y < 0.5f) y = 0.5f;
                if (y > Nfloat + 0.5f) y = Nfloat + 0.5f;
                j0 = Mathf.Floor(y);
                j1 = j0 + 1.0f;

                s1 = x - i0;
                s0 = 1.0f - s1;
                t1 = y - j0;
                t0 = 1.0f - t1;

                int i0i = (int)i0;
                int i1i = (int)i1;
                int j0i = (int)j0;
                int j1i = (int)j1;

                d[IX(i, j)] =

                    s0 * (t0 * d0[IX(i0i, j0i)] + t1 * d0[IX(i0i, j1i)])
                  + s1 * (t0 * d0[IX(i1i, j0i)] + t1 * d0[IX(i1i, j1i)]);
            }
        }
        set_bnd(b, d, N);
    }

    public void updateColors(Fluid fluid)
    {
        for (int i = 0; i < colors.Length; i++)
        {
            float r = Mathf.Sqrt(fluid.curDensityR[i]);
            float g = Mathf.Sqrt(fluid.curDensityG[i]);
            float b = Mathf.Sqrt(fluid.curDensityB[i]);
            colors[i] = new Color(r, g, b, 1);
        }
    }

}

public class Fluid
{
    public int size;
    public float dt;

    public float diffuse;
    public float viscosity;

    public float[] oldVx;
    public float[] curVx;
    public float[] oldVy;
    public float[] curVy;

    public float[] oldDensityR;
    public float[] curDensityR;

    public float[] oldDensityG;
    public float[] curDensityG;

    public float[] oldDensityB;
    public float[] curDensityB;

    public Fluid(int _size, float _dt, float _diffuse, float _viscosity)
    {
        size = _size;
        dt = _dt;
        diffuse = _diffuse;
        viscosity = _viscosity;

        oldVx = new float[_size * _size];
        curVx = new float[_size * _size];
        oldVy = new float[_size * _size];
        curVy = new float[_size * _size];
        oldDensityR = new float[_size * _size];
        curDensityR = new float[_size * _size];

        oldDensityG = new float[_size * _size];
        curDensityG = new float[_size * _size];

        oldDensityB = new float[_size * _size];
        curDensityB = new float[_size * _size];
    }

    public int IX(int x, int y)
    {
        return x + y * size;
    }

    public void AddDensity(int x, int y, float amount, int num)
    {
        if (num == 0) curDensityR[IX(x, y)] += amount;
        else if (num == 1) curDensityG[IX(x, y)] += amount;
        else if (num == 2) curDensityB[IX(x, y)] += amount;
    }

    public void AddVelocity(int x, int y, float amountX, float amountY)
    {
        curVx[IX(x, y)] += amountX;
        curVy[IX(x, y)] += amountY;
    }
}
