using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;
using TMPro;

public class Hand : MonoBehaviour
{

	[SerializeField] private AnimationCurve forceCurve;

	private struct GrabStruct
	{
		public Collider2D grabbed_Collider;
		public FixedJoint2D joint;
	}
	[Header("Hammer Settings")]
	[HideInInspector] private Rigidbody2D attachedBodyRb;
	[HideInInspector] private Transform pivotPoint;

	public Rigidbody2D HandRb { get; private set; }
	public Transform centerOfMass;
	private List<GrabStruct> grabJoints = new();
	public float hammerOffsetDistance = 5f;
	public Vector2 handOffsetNormalized;
	public float gripForce;
	public float gripStrength = 10f;
	public float stiffness = 200f;
	public float damping = 5f;
	public float maxForce = 1000f;

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
		Keyframe[] keys = forceCurve.keys;
		Keyframe k = keys[1];
		k.value = stiffness;
		k.time = hammerOffsetDistance;
		keys[1] = k;
		forceCurve.keys = keys;

		// foreach (var key in forceCurve.keys)
		// {
		// 	Debug.Log($"Key time: {key.time}, value: {key.value}");
		// }
	}
	void FixedUpdate()
	{
		ApplyHammerForces();
		ManageGrabing();
	}
	private void ApplyHammerForces()
	{
		ApplyRelativeOffsetForces(attachedBodyRb, HandRb,
									handOffsetNormalized * hammerOffsetDistance,
									pivotPoint.position, HandRb.worldCenterOfMass,
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
			Vector2 springForce = forceCurve.Evaluate(error.magnitude) * error.normalized;
			Vector2 relVel = velB - velA;
			Vector2 dampingForce = damping * relVel;

			Vector2 forceOnB = springForce - dampingForce;
			// Clamp to max force
			if (forceOnB.magnitude > maxForce)
				forceOnB = forceOnB.normalized * maxForce;
			bodyA.AddForceAtPosition(-forceOnB, posA);
			bodyB.AddForceAtPosition(forceOnB, posB);

			void ManageTorque()
			{
				Vector2 r = posB - posA;
				float torqueOnHammer = r.x * forceOnB.y - r.y * forceOnB.x;
				// Debug.Log($"Torque on hammer: {torqueOnHammer}");
				bodyA.AddTorque(-torqueOnHammer);

			}
		}
	}
	private void ManageGrabing()
	{
		CheckGrab(HandRb, ref grabJoints);
		// Remove joints if grip force is released
		foreach (var grabJoint in grabJoints.ToArray())
		{
			// Debug.Log($"Reaction Force Magnitude: {grabJoint.joint.reactionForce.magnitude}, Grip Strength * Grip Force: {gripStrength * gripForce}");
			if (grabJoint.joint.reactionForce.magnitude > gripStrength * gripForce)
			{
				Destroy(grabJoint.joint);
				grabJoints.Remove(grabJoint);
			}
		}

		void CheckGrab(Rigidbody2D hammerRb, ref List<GrabStruct> existingGrabJoints)
		{
			var collider = HandRb.GetComponent<Collider2D>();
			// var contacts = collider.(ContactPoint2D contacts);
			var contactPoints = new ContactPoint2D[12];
			int connisionCount = HandRb.GetContacts(contactPoints);
			for (int i = 0; i < connisionCount; i++)
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
				if (otherRb != null)
				{
					joint.connectedBody = otherRb;
					joint.connectedAnchor = joint.connectedBody.transform.InverseTransformPoint(contactPoint);
				}
				else
				{
					joint.connectedBody = null;
					joint.connectedAnchor = joint.transform.InverseTransformPoint(contactPoint);;
				}
			}
		}
	}
	private void InitializeComponents()
	{
		HandRb = GetComponent<Rigidbody2D>();
		for (int i = 0; i < transform.childCount; i++)
		{
			if (transform.GetChild(i).name == "centerOfMass")
				centerOfMass = transform.GetChild(i);
		}
		if (centerOfMass == null)
			Debug.LogWarning("Center of Mass not assigned and not found as child named 'centerOfMass'. Using first child as center of mass.");
	}
}
