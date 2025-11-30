using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;
using TMPro;
using Unity.Mathematics;

public class Hand : MonoBehaviour
{

	[SerializeField] private AnimationCurve forceCurve;
	[SerializeField] private AnimationCurve lenghtForceCurve;
	[SerializeField] private AnimationCurve dampeningCurve;
	public LineRenderer line { get; private set; }

	private struct GrabStruct
	{
		public Collider2D grabbed_Collider;
		public FixedJoint2D joint;
	}
	[Header("Hammer Settings")]
	public Rigidbody2D attachedBodyRb { get; private set; }
	public Rigidbody2D handRb { get; private set; }
	public Transform pivotPoint { get; private set; }
	public Transform centerOfMass { get; private set; }
	private List<GrabStruct> grabJoints = new();
	public float hammerOffsetDistance = 5f;
	public Vector2 handOffset;
	public float gripForce;
	public float gripStrength = 10f;
	public float stiffness = 200f;
	public float dampening_VelBasd = 5f;
	public float dampening_ErrorBased = 5f;
	public float maxForce = 1000f;
	public float jointBreakTorque = 0f;
	[Header("Rotation Settings")]
	public float rotationProportional = 5f;
	public float rotationDerivitive = 1f;
	public float maxRotationTorque = 100f;
	public float maxAngularSpeed = 0f; // 0 = no limit

	public void HandConstructor(Rigidbody2D attachedBodyRb, Transform pivotPoint)
	{
		if (attachedBodyRb == null)
			Debug.LogError("Attached Body Rigidbody is null in Hand constructor.");
		if (pivotPoint == null)
			Debug.LogError("Pivot Point Transform is null in Hand constructor.");
		this.attachedBodyRb = attachedBodyRb;
		this.pivotPoint = pivotPoint;
	}
	void Start()
	{
		InitializeComponents();
	}

	// Update is called once per frame
	void LateUpdate()
	{
		RenderLine();
		// Keyframe[] keys = forceCurve.keys;
		// Keyframe k = keys[1];
		// k.value = stiffness;
		// k.time = hammerOffsetDistance;
		// keys[1] = k;
		// forceCurve.keys = keys;

		// foreach (var key in forceCurve.keys)
		// {
		// 	Debug.Log($"Key time: {key.time}, value: {key.value}");
		// }
	}
	void FixedUpdate()
	{
		ApplyHammerForces();
		ManageGrabing();
		ManageRotation();
	}
	private void ApplyHammerForces()
	{
		ApplyRelativeOffsetForces(attachedBodyRb, handRb,
									handOffset * hammerOffsetDistance,
									pivotPoint.position, centerOfMass.position,
									stiffness, dampening_VelBasd, dampening_ErrorBased, maxForce);
		// ApplyRelativeLengthOffsetForces(attachedBodyRb, handRb,
		// 							hammerOffsetDistance * handOffset.magnitude,
		// 							pivotPoint.position, centerOfMass.position,
		// 							stiffness, dampening_ErrorBased, dampening_VelBasd, maxForce);

		void ApplyRelativeOffsetForces(Rigidbody2D bodyA, Rigidbody2D bodyB,
										Vector2 desiredOffset,
										Vector2 forceOffsetA, Vector2 forceOffsetB,
										float stiffness, float dampening_VelBased, float damping_ErrorBased, float maxForce)
		{
			Vector2 posA = forceOffsetA;
			Vector2 posB = forceOffsetB;
			Vector2 velA = bodyA.GetPointVelocity(posA);
			Vector2 velB = bodyB.GetPointVelocity(posB);
			Vector2 rbATargetPos = posA + desiredOffset;

			// Debug.DrawLine(posA, rbATargetPos, Color.green);
			// Debug.DrawLine(posB, posA, Color.red);


			Vector2 error = rbATargetPos - posB;
			Vector2 springForce = stiffness * error.normalized;
			Vector2 relVel = velB - velA;
			Vector2 dampingForce = (springForce.magnitude / 200) * relVel.normalized * ((relVel.magnitude * dampening_VelBased) + (damping_ErrorBased / math.min(.1f, error.magnitude)));
			// Debug.DrawLine(posB, posB - dampingForce / 300, Color.yellow);


			Vector2 forceOnB = springForce - dampingForce;
			// Clamp to max force
			if (forceOnB.magnitude > maxForce)
				forceOnB = forceOnB.normalized * maxForce;
			bodyA.AddForceAtPosition(-forceOnB, posA);
			bodyB.AddForceAtPosition(forceOnB, posB);

			// void ManageTorque()
			// {
			// 	Vector2 r = posB - posA;
			// 	float torqueOnHammer = r.x * forceOnB.y - r.y * forceOnB.x;
			// 	// Debug.Log($"Torque on hammer: {torqueOnHammer}");
			// 	bodyA.AddTorque(-torqueOnHammer);

			// }
		}
		void ApplyRelativeLengthOffsetForces(Rigidbody2D bodyA, Rigidbody2D bodyB,
										float desiredLength,
										Vector2 forceOffsetA, Vector2 forceOffsetB,
										float stiffness, float dampening_ErrorBased, float dampening_VelBased, float maxForce)
		{
			Vector2 posA = forceOffsetA;
			Vector2 posB = forceOffsetB;
			Vector2 velA = bodyA.GetPointVelocity(posA);
			Vector2 velB = bodyB.GetPointVelocity(posB);

			float error = Vector2.Distance(posA, posB) - desiredLength;

			// Debug.DrawLine(posA, posA + (posB - posA).normalized * desiredLength, Color.green);
			// Debug.DrawLine(posB, posB + (posB - posA).normalized * error, Color.red);

			Vector2 springForce = lenghtForceCurve.Evaluate(error) * (posA - posB).normalized;
			Vector2 relVel = velB - velA;
			Vector2 dampingForce = relVel.normalized * ((relVel.magnitude * dampening_VelBased) + (dampening_ErrorBased / math.min(.1f, error)));
			// Debug.DrawLine(posB, posB - dampingForce / 300, Color.yellow);


			Vector2 forceOnB = springForce - dampingForce;
			// Clamp to max force
			// if (forceOnB.magnitude > maxForce)
			// 	forceOnB = forceOnB.normalized * maxForce;
			bodyA.AddForceAtPosition(-forceOnB, posA);
			bodyB.AddForceAtPosition(forceOnB, posB);
		}
	}
	private void ManageRotation()
	{
		float targetAngle = Mathf.Atan2(handOffset.y, handOffset.x) * Mathf.Rad2Deg;
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

			// Convert angular velocity (deg/s) from radians/s returned by Rigidbody2D
			float angularVelocityDeg = rbToRotate.angularVelocity; // Rigidbody2D.angularVelocity is in degrees/sec

			// PD controller: torque = kP*error - kD*angularVelocity
			float torque = proportional * error - dirivitive * angularVelocityDeg;

			// Clamp torque
			torque = Mathf.Clamp(torque, -maxTorque, maxTorque);

			// Optionally clamp angular speed (deg/s)
			// if (maxAngularSpeed > 0f && Mathf.Abs(angularVelocityDeg) > maxAngularSpeed)
			// {
			// 	// apply counter torque to reduce angular speed
			// 	float brake = -Mathf.Sign(angularVelocityDeg) * maxTorque;
			// 	rbToRotate.AddTorque(brake);
			// }

			rbToRotate.AddTorque(torque);
			Debug.Log($"Hand Rotate Target Angle: {targetAngle}, Current Angle: {currentAngle}, Error: {error}, Angular Velocity Deg: {angularVelocityDeg}, Applied Torque: {torque}");
			Debug.DrawLine(rbToRotate.position, rbToRotate.position + Vector2.up * 2, Color.red);
			Debug.DrawLine(rbToRotate.position, rbToRotate.position + Vector2.up * torque / maxTorque * 2, Color.green);
		}
	}
	private void ManageGrabing()
	{
		if (grabJoints.Count == 0)
			CheckGrab(handRb, ref grabJoints);
		// Remove joints if grip force is released
		foreach (var grabJoint in grabJoints.ToArray())
		{
			// Debug.Log($"Reaction Force Magnitude: {grabJoint.joint.reactionForce.magnitude}, Grip Strength * Grip Force: {gripStrength * gripForce}");
			if (grabJoint.joint == null)
			{
				grabJoints.Remove(grabJoint);
				continue;
			}
			grabJoint.joint.breakForce = gripStrength * gripForce;
		}

		void CheckGrab(Rigidbody2D hammerRb, ref List<GrabStruct> existingGrabJoints)
		{
			if (gripStrength * gripForce == 0)
				return;
			// Check for collisions
			var collider = handRb.GetComponent<Collider2D>();
			// var contacts = collider.(ContactPoint2D contacts);
			var contactPoints = new ContactPoint2D[16];
			int connisionCount = handRb.GetContacts(contactPoints);
			for (int i = 0; i < math.min(1, connisionCount); i++)
			{
				ContactPoint2D contact = contactPoints[i];
				var contactCollider = contact.collider;
				if (existingGrabJoints.Exists(j => j.grabbed_Collider == contactCollider))
					continue; // Already grabbed this object
				var contactPoint = contact.point;

				var joint = gameObject.AddComponent<FixedJoint2D>();
				GrabStruct grabStruct = new GrabStruct
				{
					grabbed_Collider = contactCollider,
					joint = joint
				};
				grabJoints.Add(grabStruct);

				joint.autoConfigureConnectedAnchor = false;
				joint.enableCollision = true;
				var otherRb = contactCollider.attachedRigidbody;
				joint.anchor = joint.transform.InverseTransformPoint(contactPoint);
				joint.breakForce = gripStrength * gripForce;
				if (otherRb != null)
				{
					joint.connectedBody = otherRb;
					joint.connectedAnchor = joint.connectedBody.transform.InverseTransformPoint(contactPoint);
				}
				else
				{
					joint.connectedAnchor = joint.transform.InverseTransformPoint(contactPoint); ;
				}
			}
		}
	}
	private void InitializeComponents()
	{
		handRb = GetComponent<Rigidbody2D>();
		for (int i = 0; i < transform.childCount; i++)
		{
			if (transform.GetChild(i).name == "centerOfMass")
				centerOfMass = transform.GetChild(i);
		}
		if (centerOfMass == null)
			Debug.LogWarning("Center of Mass not assigned and not found as child named 'centerOfMass'. Using first child as center of mass.");
		line = GetComponent<LineRenderer>();
	}
	private void RenderLine()
	{
		line.SetPosition(1, transform.position);
		line.SetPosition(0, pivotPoint.position);
	}
}
