using System.Data;
using UnityEngine;
using UnityEngine.InputSystem;

public class Glider : MonoBehaviour
{
	public Rigidbody2D rb;
	[Header("Gliding Settings")]
	public float glidingCoefficient = 1;
	public float glidingEffizency = 1f;
	[Header("Boost Settings")]
	private InputAction boost;
	public float boostStrength = 10f;
	void Start()
	{
		rb = GetComponent<Rigidbody2D>();
		boost = InputSystem.actions.FindAction("Boost");
	}
	void FixedUpdate()
	{
		ApplyGlidingForce();
		Boost();
	}
	private void Boost()
	{
		rb.AddForce(transform.right * boost.ReadValue<float>() * boostStrength);
	}
	private void ApplyGlidingForce()
	{
		Vector2 horizontalVel = Vector3.Dot(rb.linearVelocity, transform.right) * transform.right;
		// Vector2 horizontalVelDir = horizontalVel.normalized;

		Vector2 verticalVel = Vector3.Dot(rb.linearVelocity, transform.up) * transform.up;
		// Vector2 verticalVelDir = verticalVel.normalized;
		// float verticalSpeed = verticalVel.magnitude;

		Vector2 glideVel = -ReflectVector(verticalVel, transform.up).normalized * glidingCoefficient * verticalVel.magnitude * verticalVel.magnitude;

		rb.AddForce(glideVel * glidingEffizency, ForceMode2D.Impulse);
		Debug.DrawLine(transform.position, transform.position + (Vector3)glideVel, Color.red);
	}

	public Vector3 ReflectVector(Vector3 v, Vector3 reflactionAxisNormalized)
	{
		return 2f * Vector3.Dot(v, reflactionAxisNormalized) * reflactionAxisNormalized - v;
	}
}
