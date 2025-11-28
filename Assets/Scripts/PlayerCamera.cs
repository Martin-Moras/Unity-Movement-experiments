using UnityEngine;

public class PlayerCamera : MonoBehaviour
{
	public Camera thisCamera;
	public Transform target;
	public float size = 10f;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
		thisCamera = GetComponent<Camera>();
		thisCamera.orthographicSize = size;
    }

    // Update is called once per frame
    void Update()
    {
        thisCamera.transform.position = target.position;
		thisCamera.orthographicSize = size;
    }
}
