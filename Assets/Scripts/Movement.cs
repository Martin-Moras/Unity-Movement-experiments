using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using Mono.Cecil.Cil;
using UnityEngine;
using UnityEngine.InputSystem;

public class Movement : MonoBehaviour
{
	[HideInInspector] public Rigidbody2D rb;
	#region Input Actions
	[HideInInspector] public InputAction reset;
	[HideInInspector] public InputAction boost;
	[HideInInspector] public InputAction grab_l;
	[HideInInspector] public InputAction grab_r;
	[HideInInspector] public InputAction move;
	[HideInInspector] public InputAction look;
	#endregion
	#region Prefabs
	public GameObject handPrefab;

	#endregion
	[Header("Hammer Settings")]
	public Hand Hand_L;
	public Hand Hand_R;
	[HideInInspector] public Transform handPivotPoint_L;
	[HideInInspector] public Transform handPivotPoint_R;
	public float hammerOffsetDistance = 2f;
	public float stiffness = 200f;
	public float damping = 20f;
	public float maxForce = 1000f;

	[Header("Other Settings")]
	public Transform centerOfMass;
	public float speed = 5f;
	public float rotationSpeed = 5f;
	// Start is called once before the first execution of Update after the MonoBehaviour is created
	void Start()
	{
		InitializeVariables();
		Physics2D.IgnoreCollision(Hand_L.GetComponent<Collider2D>(), Hand_R.GetComponent<Collider2D>());
		Physics2D.IgnoreCollision(rb.GetComponent<Collider2D>(), Hand_R.GetComponent<Collider2D>());
		Physics2D.IgnoreCollision(Hand_L.GetComponent<Collider2D>(), rb.GetComponent<Collider2D>());
	}

	// Update is called once per frame
	void FixedUpdate()
	{
		Hand_L.handOffsetNormalized = move.ReadValue<Vector2>().normalized;
		Hand_L.gripForce = grab_l.ReadValue<float>();

		Hand_R.handOffsetNormalized = look.ReadValue<Vector2>().normalized;
		Hand_R.gripForce = grab_r.ReadValue<float>();
	}
	void LateUpdate()
	{
		HandleReset();
	}
	private void HandleReset()
	{
		if (reset.triggered)
		{
			Rigidbody2D[] rbs = { rb, Hand_L.GetComponent<Rigidbody2D>(), Hand_R.GetComponent<Rigidbody2D>() };
			foreach (Rigidbody2D rigidbody in rbs)
			{
				rigidbody.linearVelocity = Vector2.zero;
				rigidbody.angularVelocity = 0f;
				rigidbody.transform.position = Vector3.zero;
				rigidbody.transform.rotation = Quaternion.identity;
			}
		}
	}
	private void InitializeVariables()
	{
		rb = GetComponent<Rigidbody2D>();
		// find center of mass and hands if not assigned
		for (int i = 0; i < transform.childCount; i++)
		{
			if (transform.GetChild(i).name == "centerOfMass")
				centerOfMass = transform.GetChild(i);
		}
		// find hands in center of mass children
		for (int i = 0; i < centerOfMass.childCount; i++)
		{
			if (centerOfMass.GetChild(i).name == "Hand_L")
				handPivotPoint_L = centerOfMass.GetChild(i).transform;
			else if (centerOfMass.GetChild(i).name == "Hand_R")
				handPivotPoint_R = centerOfMass.GetChild(i).transform;
		}
		if (centerOfMass == null)
			Debug.LogWarning("Center of Mass not assigned and not found as child named 'centerOfMass'. Using first child as center of mass.");
		centerOfMass = transform.GetChild(0);
		InitializeHands();
		// Initialize Input Actions
		move = InputSystem.actions.FindAction("Move");
		look = InputSystem.actions.FindAction("Look");
		reset = InputSystem.actions.FindAction("Reset");
		boost = InputSystem.actions.FindAction("Boost");
		grab_r = InputSystem.actions.FindAction("Grab_R");
		grab_l = InputSystem.actions.FindAction("Grab_L");
		void InitializeHands()
		{
			Hand_L = InitHand("Hand_L", handPivotPoint_L);
			Hand_R = InitHand("Hand_R", handPivotPoint_R);

			Hand InitHand(string handName, Transform pivotPoint)
			{
				GameObject hand = Instantiate(handPrefab, pivotPoint.position, Quaternion.identity);
				var handScript = hand.GetComponent<Hand>();
				handScript.HandConstructor(rb, pivotPoint);
				hand.name = handName;

				return handScript;
			}
		}
	}
}
