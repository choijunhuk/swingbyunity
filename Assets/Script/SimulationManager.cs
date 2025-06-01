using System;
using UnityEngine;

[DisallowMultipleComponent]
public class SimulationManager : MonoBehaviour
{
    public static SimulationManager Instance { get; private set; }

    [Header("Prefab & References")]
    [SerializeField] private GameObject shipPrefab;
    [SerializeField] private CelestialBody planetBody;
    [SerializeField] private Transform planetTransform;

    [Header("우주선 파라미터 (UI 입력, 단위: km)")]
    public float shipMass = 1e4f;
    public Vector3 shipPos = new Vector3(-500f, 0f, 0f);
    public Vector3 shipVel = new Vector3(5f, 0f, 0f);
    public float shipRadius = 0.01f;

    [Header("행성 파라미터 (UI 입력, 단위: km)")]
    public float planetMass = 5.972e24f;
    public Vector3 planetPos = new Vector3(0f, 0f, 0f);
    public Vector3 planetVel = new Vector3(0f, 0f, 0f);
    public float planetRadius = 0f;
    public bool isPointMass = true;

    [Header("물리 상수 & 설정")]
    public float gravConst = 6.674e-11f;
    public float timeScale = 3.0f;
    public float trajWidth = 0.1f;

    [Header("해석적 vs 수치 통합")]
    [Tooltip("0: 해석적, 1: 수치")]
    public int simulationMethod = 0;

    [Header("Trail Colors")]
    public Color shipTrailColor = Color.cyan;
    public Color planetTrailColor = Color.yellow;

    [Header("카메라 모드")]
    public int camMode = 0;

    public event Action<string> OnLogUpdated;
    public event Action<float> OnSpeedUpdated;

    private GameObject shipInstance;
    private TrailRenderer shipTrail, planetTrail;
    private float mu;
    private float a;
    private float e;
    private float omega;
    private float theta;
    private float r0Mag;
    private Vector3 r0;
    private Vector3 v0;
    private float M0;
    private float n;
    private float elapsedTime;
    private bool running;
    private bool analyticEllipse;
    private Vector3 pos_m;
    private Vector3 vel_m;
    private Vector3 planetPos_m;

    public bool IsRunning => running;
    public Transform ShipTransform => shipInstance?.transform;
    public Transform PlanetTransform => planetTransform;

    void Awake()
    {
        if (Instance == null) Instance = this;
        else Destroy(gameObject);
        Debug.Log("[SimulationManager] Awake: Singleton 초기화 완료");
    }

    public void StartSimulation()
    {
        ClearPrevious();
        elapsedTime = 0f;
        analyticEllipse = false;

        float unit = 1e3f; // 1km = 1000m
        mu = gravConst * planetMass;

        planetPos_m = planetPos * unit;
        planetTransform.position = (planetPos_m / 100f) / 10f; // 포지션 값을 10으로 나눔
        planetTransform.localScale = Vector3.one;
        Debug.Log($"[SimulationManager] StartSimulation: planetPos={planetPos}, planetPos_m={planetPos_m}, planetTransform.position={planetTransform.position}, planetVel={planetVel}, planetMass={planetMass}, planetRadius={planetRadius}");

        if (!planetBody.TryGetComponent<TrailRenderer>(out planetTrail))
            planetTrail = planetBody.gameObject.AddComponent<TrailRenderer>();
        planetTrail.widthMultiplier = trajWidth;
        planetTrail.time = 100f;
        planetTrail.minVertexDistance = 0.2f;
        planetTrail.startColor = planetTrail.endColor = planetTrailColor;
        planetTrail.Clear();
        planetTrail.emitting = true;

        shipInstance = Instantiate(shipPrefab);
        if (shipInstance == null) Debug.LogError("[SimulationManager] 우주선 생성 실패!");
        shipInstance.transform.localScale = Vector3.one;
        pos_m = shipPos * unit;
        shipInstance.transform.position = (pos_m / 100f) / 10f; // 포지션 값을 10으로 나눔
        vel_m = shipVel * unit;
        Debug.Log($"[SimulationManager] 우주선 생성: shipMass={shipMass}, shipPos={shipPos}, shipVel={shipVel}, shipRadius={shipRadius}, shipTransform.position={shipInstance.transform.position}");

        if (!shipInstance.TryGetComponent<TrailRenderer>(out shipTrail))
            shipTrail = shipInstance.AddComponent<TrailRenderer>();
        shipTrail.widthMultiplier = trajWidth;
        shipTrail.time = 100f;
        shipTrail.minVertexDistance = 0.2f;
        shipTrail.startColor = shipTrail.endColor = shipTrailColor;
        shipTrail.Clear();
        shipTrail.emitting = true;

        Collider shipCol = shipInstance.GetComponent<Collider>();
        Collider planetCol = planetBody.GetComponent<Collider>();
        if (shipCol != null && planetCol != null)
            Physics.IgnoreCollision(shipCol, planetCol, true);

        if (simulationMethod == 0)
        {
            r0 = shipPos * unit - planetPos_m;
            v0 = shipVel * unit - planetVel * unit;
            r0Mag = r0.magnitude;

            float v0sq = v0.sqrMagnitude;
            float energy = 0.5f * v0sq - mu / r0Mag;

            float Lmag = Vector3.Cross(r0, v0).magnitude;
            if (Lmag < 1e-6f)
            {
                analyticEllipse = false;
                running = true;
                OnLogUpdated?.Invoke("--- 진동 또는 방사형 궤도: 수치 모드로 전환 ---");
                Debug.Log("[SimulationManager] 진동/방사형 궤도: 수치 모드로 전환");
            }
            else if (energy < 0f)
            {
                analyticEllipse = true;
                Vector3 eVec = ((v0sq - mu / r0Mag) * r0 - Vector3.Dot(r0, v0) * v0) / mu;
                e = eVec.magnitude;

                float rawOmega = Mathf.Atan2(eVec.y, eVec.x);
                if (rawOmega < 0f) rawOmega += 2f * Mathf.PI;
                omega = rawOmega;

                a = -mu / (2f * energy);
                n = Mathf.Sqrt(mu / (a * a * a));

                float cosTheta0 = Vector3.Dot(eVec, r0) / (e * r0Mag);
                cosTheta0 = Mathf.Clamp(cosTheta0, -1f, 1f);
                float theta0 = Mathf.Acos(cosTheta0);
                if (Vector3.Dot(r0, v0) < 0f)
                    theta0 = 2f * Mathf.PI - theta0;

                float tanHalfTheta0 = Mathf.Tan(theta0 / 2f);
                float sqrtFactor = Mathf.Sqrt((1f - e) / (1f + e));
                float E0 = 2f * Mathf.Atan(sqrtFactor * tanHalfTheta0);
                if (E0 < 0f) E0 += 2f * Mathf.PI;

                M0 = E0 - e * Mathf.Sin(E0);
                theta = theta0;

                running = true;
                OnLogUpdated?.Invoke("--- 시뮬레이션 시작 (이론적·타원 궤도) ---");
                Debug.Log("[SimulationManager] 시뮬레이션 시작 (이론적·타원 궤도): a=" + a + ", e=" + e + ", omega=" + omega);
            }
            else
            {
                analyticEllipse = false;
                running = true;
                OnLogUpdated?.Invoke("--- 에너지 ≥ 0: 수치 모드로 전환 ---");
                Debug.Log("[SimulationManager] 에너지 ≥ 0: 수치 모드로 전환");
            }
        }
        else
        {
            analyticEllipse = false;
            running = true;
            OnLogUpdated?.Invoke("--- 시뮬레이션 시작 (수치) ---");
            Debug.Log("[SimulationManager] 시뮬레이션 시작 (수치)");
        }
    }

    public void StopSimulation()
    {
        running = false;
        if (shipTrail != null) shipTrail.emitting = false;
        if (planetTrail != null) planetTrail.emitting = false;
        OnLogUpdated?.Invoke("--- 시뮬레이션 정지 ---");
        Debug.Log("[SimulationManager] 시뮬레이션 정지");
    }

    public void ResumeSimulation()
    {
        running = true;
        if (shipTrail != null) shipTrail.emitting = true;
        if (planetTrail != null) planetTrail.emitting = true;
        OnLogUpdated?.Invoke("--- 시뮬레이션 재개됨 ---");
        Debug.Log("[SimulationManager] 시뮬레이션 재개됨");
    }

    private Vector3 CalculateAcceleration(Vector3 position, Vector3 velocity)
    {
        Vector3 rVec = planetPos_m - position;
        float distSqr = rVec.sqrMagnitude;
        if (distSqr > 1e-12f && !float.IsNaN(distSqr))
        {
            float dist = Mathf.Sqrt(distSqr);
            return (mu / (dist * dist)) * rVec.normalized;
        }
        Debug.LogWarning("[SimulationManager] CalculateAcceleration: Invalid distSqr=" + distSqr);
        return Vector3.zero;
    }

    void FixedUpdate()
    {
        if (!running || shipInstance == null)
        {
            Debug.LogWarning("[SimulationManager] FixedUpdate 중단: running=" + running + ", shipInstance=" + (shipInstance == null));
            return;
        }

        float dt = Time.fixedDeltaTime * timeScale;
        elapsedTime += dt;
        float unit = 1e3f;

        planetPos_m += (planetVel * unit) * dt;
        planetTransform.position = (planetPos_m / 100f) / 10f; // 포지션 값을 10으로 나눔
        Debug.Log($"[SimulationManager] FixedUpdate: planetPos_m={planetPos_m}, planetTransform.position={planetTransform.position}, dt={dt}");

        if (simulationMethod == 0 && analyticEllipse)
        {
            float M = M0 + n * elapsedTime;
            float E = M + e * Mathf.Sin(M) / (1f - Mathf.Sin(M + e) + Mathf.Sin(M));
            for (int i = 0; i < 8; i++)
            {
                float f = E - e * Mathf.Sin(E) - M;
                if (Mathf.Abs(f) < 1e-6f) break;
                float fPrime = 1f - e * Mathf.Cos(E);
                E -= f / fPrime;
            }

            float cosE = Mathf.Cos(E);
            float sinE = Mathf.Sin(E);
            float fac = Mathf.Sqrt((1f + e) / (1f - e));
            theta = 2f * Mathf.Atan2(fac * sinE, cosE);

            float r_val = a * (1f - e * cosE);
            float angle = theta + omega;
            Vector3 newPos_m = new Vector3(
                r_val * Mathf.Cos(angle),
                r_val * Mathf.Sin(angle),
                0f
            );

            Vector3 finalPos = planetPos_m + newPos_m;
            if (!float.IsNaN(finalPos.x))
            {
                shipInstance.transform.position = (finalPos / 100f) / 10f; // 포지션 값을 10으로 나눔
                float speed_m = Mathf.Sqrt(mu * (2f / r_val - 1f / a));
                float speedKm = speed_m / unit;
                float thetaNorm = theta % (2f * Mathf.PI);
                if (thetaNorm < 0f) thetaNorm += 2f * Mathf.PI;
                float thetaDeg = thetaNorm * Mathf.Rad2Deg;
                OnLogUpdated?.Invoke($"Analytic θ={thetaDeg:F2}°, v={speedKm:0.00} km/s");
                OnSpeedUpdated?.Invoke(speedKm);
                Debug.Log($"[SimulationManager] Analytic: pos={shipInstance.transform.position}, speedKm={speedKm}, thetaDeg={thetaDeg}");
            }
            else
            {
                Debug.LogWarning("[SimulationManager] Analytic: Invalid finalPos=" + finalPos);
            }
        }
        else
        {
            Vector3 rVec = planetPos_m - pos_m;
            float distSqr = rVec.sqrMagnitude;
            float speed_m = vel_m.magnitude;
            float speedKm = speed_m / unit;

            if (distSqr > 1e-12f && !float.IsNaN(distSqr))
            {
                float dist = Mathf.Sqrt(distSqr);
                float dynamicDt = dt * Mathf.Min(1f, dist / 1e6f);
                Vector3 k1_v = CalculateAcceleration(pos_m, vel_m) * dynamicDt;
                Vector3 k1_r = vel_m * dynamicDt;
                Vector3 k2_v = CalculateAcceleration(pos_m + k1_r * 0.5f, vel_m + k1_v * 0.5f) * dynamicDt;
                Vector3 k2_r = (vel_m + k1_v * 0.5f) * dynamicDt;
                Vector3 k3_v = CalculateAcceleration(pos_m + k2_r * 0.5f, vel_m + k2_v * 0.5f) * dynamicDt;
                Vector3 k3_r = (vel_m + k2_v * 0.5f) * dynamicDt;
                Vector3 k4_v = CalculateAcceleration(pos_m + k3_r, vel_m + k3_v) * dynamicDt;
                Vector3 k4_r = (vel_m + k3_v) * dynamicDt;

                vel_m += (k1_v + 2f * k2_v + 2f * k3_v + k4_v) / 6f;
                pos_m += (k1_r + 2f * k2_r + 2f * k3_r + k4_r) / 6f;

                Vector3 newPos = pos_m / 100f;
                if (!float.IsNaN(newPos.x))
                {
                    shipInstance.transform.position = newPos / 10f; // 포지션 값을 10으로 나눔
                    shipInstance.transform.rotation = Quaternion.LookRotation(vel_m, Vector3.up);
                    float ang = Mathf.Atan2(rVec.y, rVec.x) * Mathf.Rad2Deg;
                    if (ang < 0f) ang += 360f;
                    OnLogUpdated?.Invoke($"Numeric θ={ang:F2}°, v={speedKm:0.00} km/s, dist={dist / unit:F2} km");
                    OnSpeedUpdated?.Invoke(speedKm);
                    Debug.Log($"[SimulationManager] Numeric: pos={shipInstance.transform.position}, speedKm={speedKm}, theta={ang:F2}, dist={dist / unit:F2} km");
                }
                else
                {
                    Debug.LogWarning("[SimulationManager] Numeric: Invalid newPos=" + newPos);
                }
            }
            else
            {
                OnLogUpdated?.Invoke($"Numeric: Invalid dist={Mathf.Sqrt(distSqr):F2}, v={speedKm:0.00} km/s");
                OnSpeedUpdated?.Invoke(speedKm);
                Debug.LogWarning("[SimulationManager] Numeric: Invalid distSqr=" + distSqr + ", speedKm=" + speedKm);
            }
        }
    }

    private void ClearPrevious()
    {
        if (shipInstance != null)
        {
            Destroy(shipInstance);
            shipInstance = null;
            shipTrail = null;
            Debug.Log("[SimulationManager] ClearPrevious: 이전 우주선 제거");
        }
    }
}