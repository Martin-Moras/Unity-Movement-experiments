using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody2D))]
public class SimpleGlider2D : MonoBehaviour
{
    [Header("Geometry & aero")]
    public float wingArea = 1f;           // m^2 (scale for forces)
    public float CL0 = 0f;                // lift coeff at 0 AoA
    public float CLalpha = 2.0f;          // lift slope (per rad)
    public float CD0 = 0.02f;             // base drag coeff
    public float inducedFactor = 0.1f;    // simple induced drag multiplier

    [Header("Controls")]
    public float elevatorAuthority = 0.5f; // modifies effective AoA (radians)
    public float elevatorSpeed = 5f;       // how fast deflection follows input

    [Header("Environment & tuning")]
    public float airDensity = 1.225f;
    public Vector2 wind = Vector2.zero;
    public float angularDamping = 0.5f;   // reduces spinning

    Rigidbody2D rb;
    float elevator = 0f;

    void Awake()
    {
        rb = GetComponent<Rigidbody2D>();
    }

    void FixedUpdate()
    {
        // relative air velocity
        Vector2 vAir = rb.linearVelocity - wind;
        float speed = vAir.magnitude;
        if (speed < 0.0001f) return;

        // local velocity (forward = transform.right)
        float vForward = Vector2.Dot(vAir, transform.right);
        float vUp = Vector2.Dot(vAir, transform.up);

        // angle of attack (rad)
        float aoa = Mathf.Atan2(vUp, Mathf.Max(0.0001f, vForward));

        // elevator input smoothing
        float input = InputSystem.actions.FindAction("Boost").ReadValue<float>();;
        float target = Mathf.Clamp(input, -1f, 1f) * elevatorAuthority;
        elevator = Mathf.MoveTowards(elevator, target, elevatorSpeed * Time.fixedDeltaTime);
        float effectiveAoA = aoa;// + elevator;

        // coefficients
        float CL = CL0 + CLalpha * effectiveAoA;
        float CD = CD0 + inducedFactor * CL * CL;

        // dynamic pressure
        float q = 0.5f * airDensity * speed * speed;

        // forces (in world space)
        Vector2 velDir = vAir.normalized;
        Vector2 dragForce = -velDir * (q * wingArea * CD) * 0;
        Vector2 liftDir = new Vector2(-velDir.y, velDir.x); // perpendicular
        Vector2 liftForce = liftDir * (q * wingArea * CL);

        // apply forces at center (simple)
        rb.AddForce(liftForce + dragForce);

        // simple aerodynamic pitch torque: proportional to AoA and speed
        float pitchTorque = q * wingArea * (effectiveAoA) * -1f; // sign tuned for natural nose behavior
        // rb.AddTorque(pitchTorque - angularDamping * rb.angularVelocity);
    }
}
