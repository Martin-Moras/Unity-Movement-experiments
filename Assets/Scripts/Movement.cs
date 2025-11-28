using UnityEngine;
using UnityEngine.InputSystem;

public class Movement : MonoBehaviour
{
	[HideInInspector] public Rigidbody2D rb;
	[HideInInspector] public InputAction move;
	[HideInInspector] public InputAction look;
	[HideInInspector] public InputAction reset;
	[HideInInspector] public InputAction boost;
	[Header("Hammer Settings")]
	public Rigidbody2D hammerL_Rb;
	public Rigidbody2D hammerR_Rb;
	[HideInInspector] public Transform hand_L;
	[HideInInspector] public Transform hand_R;
	public float hammerOffsetDistance = 2f;
	public float stiffness = 200f;
	public float damping = 20f;
	public float maxForce = 1000f;
	[Header("Gliding Settings")]
	public float glidingCoefficient = 1;
	public float glidingEffizency = 1f;
	[Header("Other Settings")]
	public Transform centerOfMass;
	public float speed = 5f;
	public float rotationSpeed = 5f;
	// Start is called once before the first execution of Update after the MonoBehaviour is created
	void Start()
	{
		InitializeComponents();
		Physics2D.IgnoreCollision(hammerL_Rb.GetComponent<Collider2D>(), hammerR_Rb.GetComponent<Collider2D>());
		Physics2D.IgnoreCollision(rb.GetComponent<Collider2D>(), hammerR_Rb.GetComponent<Collider2D>());
		Physics2D.IgnoreCollision(hammerL_Rb.GetComponent<Collider2D>(), rb.GetComponent<Collider2D>());
	}

	// Update is called once per frame
	void FixedUpdate()
	{
		//rb.centerOfMass = centerOfMass.localPosition;
		ApplyBoost();
		//ApplyGlidingForce();
		ApplyHammerForces();
	}
	void LateUpdate()
	{
		HandleReset();

	}
	private void ApplyGlidingForce()
	{
		Vector2 horizontalVel = Vector3.Dot(rb.linearVelocity, transform.right) * transform.right;
		Vector2 horizontalVelDir = horizontalVel.normalized;

		Vector2 verticalVel = Vector3.Dot(rb.linearVelocity, transform.up) * transform.up;
		Vector2 verticalVelDir = verticalVel.normalized;
		float verticalSpeed = verticalVel.magnitude;

		Vector2 glideVel = Vector2.Dot(transform.right, verticalVel) * transform.right * glidingCoefficient;

		rb.AddForce(-verticalVel * glidingCoefficient);
		rb.AddForce(glideVel * glidingEffizency);
		Debug.DrawLine(transform.position, transform.position + (Vector3)glideVel, Color.red);
	}
	private void ApplyHammerForces()
	{
		ApplyRelativeOffsetForces(rb, hammerL_Rb, 
									move.ReadValue<Vector2>() * hammerOffsetDistance,
									hand_L.position, hammerL_Rb.worldCenterOfMass,
									stiffness, damping, maxForce);
		ApplyRelativeOffsetForces(rb, hammerR_Rb,
									look.ReadValue<Vector2>() * hammerOffsetDistance,
									hand_R.position, hammerR_Rb.worldCenterOfMass,
									stiffness, damping, maxForce);

		void ApplyRelativeOffsetForces(Rigidbody2D bodyA, Rigidbody2D bodyB,
										Vector2 desiredOffset, 
										Vector2 forceOffsetA, Vector2 forceOffsetB,
										float stiffness = 200f, float damping = 20f, float maxForce = 1000f)
		{
			Vector2 posA = forceOffsetA;
			Vector2 posB = forceOffsetB;
			Vector2 velA = bodyA.GetPointVelocity(posA);
			Vector2 velB = bodyB.GetPointVelocity(posB);
			Vector2 rbATargetPos = posA + desiredOffset;

			Debug.DrawLine(posA, rbATargetPos, Color.green);
			Debug.DrawLine(posB, posA, Color.red);


			Vector2 error = rbATargetPos - posB;
			Vector2 springForce = stiffness * error;
			Vector2 relVel = velB - velA;
			Vector2 dampingForce = damping * relVel;

			Vector2 forceOnB = springForce - dampingForce;
			// Clamp to max force
			if (forceOnB.magnitude > maxForce)
				forceOnB = forceOnB.normalized * maxForce;

			bodyA.AddForceAtPosition(-forceOnB, posA);
			Vector2 r = posB - posA;
			float torqueOnHammer = r.x * forceOnB.y - r.y * forceOnB.x;
			// Debug.Log($"Torque on hammer: {torqueOnHammer}");
			bodyA.AddTorque(-torqueOnHammer);
			bodyB.AddForceAtPosition(forceOnB, posB);
		}
	}

	private void ApplyBoost()
	{
		rb.AddForce(transform.right * boost.ReadValue<float>() * speed);
	}
	private void Rotate()
	{
		float angle = Mathf.Atan2(look.ReadValue<Vector2>().y, look.ReadValue<Vector2>().x) * Mathf.Rad2Deg;
		rb.rotation = angle;
	}
	private void HandleReset()
	{
		if (reset.triggered)
		{
			Rigidbody2D[] rbs = { rb, hammerL_Rb, hammerR_Rb };
			foreach (Rigidbody2D rigidbody in rbs)
			{
				rigidbody.linearVelocity = Vector2.zero;
				rigidbody.angularVelocity = 0f;
				rigidbody.transform.position = Vector3.zero;
				rigidbody.transform.rotation = Quaternion.identity;
			}
		}
	}
	private void InitializeComponents()
	{
		rb = GetComponent<Rigidbody2D>();
		for (int i = 0; i < transform.childCount; i++)
		{
			if (transform.GetChild(i).name == "centerOfMass")
				centerOfMass = transform.GetChild(i);
		}
		for (int i = 0; i < centerOfMass.childCount; i++)
		{
			if (centerOfMass.GetChild(i).name == "Hand_L")
				hand_L = centerOfMass.GetChild(i).transform;
			else if (centerOfMass.GetChild(i).name == "Hand_R")
				hand_R = centerOfMass.GetChild(i).transform;
		}
		if (centerOfMass == null)
			Debug.LogWarning("Center of Mass not assigned and not found as child named 'centerOfMass'. Using first child as center of mass.");
		centerOfMass = transform.GetChild(0);
		move = InputSystem.actions.FindAction("Move");
		look = InputSystem.actions.FindAction("Look");
		reset = InputSystem.actions.FindAction("Reset");
		boost = InputSystem.actions.FindAction("Boost");
	}
}
