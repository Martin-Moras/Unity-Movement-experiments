using System;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.InputSystem;

public class torqueControllerTest : MonoBehaviour
{
	public Vector2 handOffset;
	public Rigidbody2D handRb;
	public Transform centerOfMass;

	public float rotationProportional = 5f;
	public float rotationDerivitive = 1f;
	public float rotationIntegral = .1f;
	public float rotationTorque = .1f;
	public float maxRotationTorque = 100f;
	public float maxAngularSpeed = 0f; // 0 = no limit
	private float lastFrame_errorVelocity;
	[SerializeField] private float adaptiveAngularAcceleration;
	// Start is called once before the first execution of Update after the MonoBehaviour is created
	void Start()
	{
		handRb = GetComponent<Rigidbody2D>();
		centerOfMass = transform.GetChild(0);
	}

	// Update is called once per frame
	void Update()
	{
		handOffset = InputSystem.actions.FindAction("Move").ReadValue<Vector2>();
		handRb.centerOfMass = centerOfMass.position;
	}
	void FixedUpdate()
	{
		ManageRotation();
	}
	private void ManageRotation()
	{
		float targetAngle = Mathf.Atan2(handOffset.y, handOffset.x) * Mathf.Rad2Deg - 90;
		RotateTowards(targetAngle,
						rbToRotate: handRb,
						proportional: rotationProportional,
						dirivitive: rotationDerivitive,
						maxTorque: maxRotationTorque,
						maxAngularSpeed: maxAngularSpeed);
		void RotateTowards(float targetAngle, Rigidbody2D rbToRotate, float proportional, float dirivitive, float maxTorque, float maxAngularSpeed)
		{
			// Current angle in degrees
			float currentAngle = rbToRotate.rotation; // same as transform.eulerAngles.z but better for physics
													  // Shortest signed angle difference (-180,180]
			float error = Mathf.DeltaAngle(currentAngle, targetAngle);
			var offPos = (Vector2)transform.position + handOffset;
			var targetPos = (Vector2)transform.position + (Vector2)(Quaternion.Euler(0,0,targetAngle) * Vector2.up * 2);
			Debug.DrawLine(transform.position, offPos.normalized * 2, Color.green);
			// Convert angular velocity (deg/s) from radians/s returned by Rigidbody2D
			float angularVelocityDeg = rbToRotate.angularVelocity; // Rigidbody2D.angularVelocity is in degrees/sec

			float targetVelocity = error * math.abs(error) * proportional;

			float errorVelocity = targetVelocity - angularVelocityDeg;
			Debug.DrawLine(targetPos + (Vector2)transform.up*.1f, targetPos - errorVelocity * (Vector2)transform.right * .003f + (Vector2)transform.up*.1f, Color.red);
			Debug.DrawLine(targetPos, targetPos - targetVelocity * (Vector2)transform.right * .003f, Color.green);

			float angularAcceleration = (lastFrame_errorVelocity - angularVelocityDeg) / Time.fixedDeltaTime;
			adaptiveAngularAcceleration += (angularAcceleration) * rotationIntegral;
			Debug.DrawLine(targetPos, targetPos - adaptiveAngularAcceleration * (Vector2)transform.right, Color.yellow);

			float torque = math.abs(error) * rotationTorque + errorVelocity * (rotationDerivitive + adaptiveAngularAcceleration);
			// Clamp torque
			torque = Mathf.Clamp(torque, -maxTorque, maxTorque);

			rbToRotate.AddTorque(torque);
				Debug.Log($"Angular error: {errorVelocity}, Error: {error}, Angular Velocity Deg: {angularVelocityDeg}, Applied Torque: {torque}, adaptive accel: {adaptiveAngularAcceleration}");
			Debug.DrawLine(rbToRotate.position, rbToRotate.position + Vector2.up * 2, Color.red);
			Debug.DrawLine(rbToRotate.position, rbToRotate.position + Vector2.up * torque / maxTorque * 2, Color.green);

			lastFrame_errorVelocity = errorVelocity;
		}
	}
}
