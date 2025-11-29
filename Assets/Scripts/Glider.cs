using UnityEngine;

public class Glider : MonoBehaviour
{
	public Rigidbody2D rb;
	[Header("Gliding Settings")]
	public float glidingCoefficient = 1;
	public float glidingEffizency = 1f;
	void Start()
	{
		rb = GetComponent<Rigidbody2D>();
	}

	void FixedUpdate()
	{
		ApplyGlidingForce();
	}
	private void ApplyGlidingForce()
	{
		Vector2 horizontalVel = Vector3.Dot(rb.linearVelocity, transform.right) * transform.right;
		// Vector2 horizontalVelDir = horizontalVel.normalized;

		Vector2 verticalVel = Vector3.Dot(rb.linearVelocity, transform.up) * transform.up;
		// Vector2 verticalVelDir = verticalVel.normalized;
		// float verticalSpeed = verticalVel.magnitude;

		Vector2 glideVel = Vector2.Dot(transform.right, verticalVel) * transform.right * glidingCoefficient;

		rb.AddForce(-verticalVel * glidingCoefficient);
		rb.AddForce(glideVel * glidingEffizency);
		Debug.DrawLine(transform.position, transform.position + (Vector3)glideVel, Color.red);
	}
}
