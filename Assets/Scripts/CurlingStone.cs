using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Random = UnityEngine.Random;

public class CurlingStone : Agent
{
    private Rigidbody rbody;
    public Transform target;

    private void Start()
    {
        rbody = GetComponent<Rigidbody>();
        Time.fixedDeltaTime = 0.01f;
    }

    private void FixedUpdate()
    {
        var v = rbody.velocity;
        const float airDragMultiplier = -0.005f;
        var airDrag = new Vector3(airDragMultiplier * v.x * Math.Abs(v.x), 0.0f,
            airDragMultiplier * v.z * Math.Abs(v.z));
        rbody.AddForce(airDrag);
    }

    private void OnTriggerEnter(Collider other)
    {
        const float collideWallMultiplier = -0.9f;
        var v = rbody.velocity;
        if (other.gameObject.CompareTag("VerticalWall"))
        {
            if (transform.localPosition.x > 49.0f && rbody.velocity.x > 0.0f)
            {
                var v1 = rbody.velocity;
                var p = rbody.position;
                rbody.velocity = new Vector3(collideWallMultiplier * v1.x, v1.y, v1.z);
                rbody.position = new Vector3(p.x - transform.localPosition.x + 48.75f, p.y, p.z);
            }
            else if (transform.localPosition.x < -49.0f && rbody.velocity.x < 0.0f)
            {
                var v1 = rbody.velocity;
                var p = rbody.position;
                rbody.velocity = new Vector3(collideWallMultiplier * v1.x, v1.y, v1.z);
                rbody.position = new Vector3(p.x - transform.localPosition.x - 48.75f, p.y, p.z);
            }
            else
            {
                // rbody.AddForce(new Vector3(collideWallMultiplier * v.x, 0.0f, 0.0f), ForceMode.VelocityChange);
                rbody.velocity = new Vector3(collideWallMultiplier * v.x, v.y, v.z);
            }
        }
        else if (other.gameObject.CompareTag("HorizontalWall"))
        {
            if (transform.localPosition.z > 49.0f && rbody.velocity.z > 0.0f)
            {
                var v1 = rbody.velocity;
                rbody.velocity = new Vector3(v1.x, v1.y, collideWallMultiplier * v1.z);
                var p = rbody.position;
                rbody.position = new Vector3(p.x, p.y, p.z - transform.localPosition.z + 48.75f);
            }
            else if (transform.localPosition.z < -49.0f && rbody.velocity.z < 0.0f)
            {
                var v1 = rbody.velocity;
                rbody.velocity = new Vector3(v1.x, v1.y, collideWallMultiplier * v1.z);
                var p = rbody.position;
                rbody.position = new Vector3(p.x, p.y, p.z - transform.localPosition.z - 48.75f);
            }
            else
            {
                // rbody.AddForce(new Vector3(0.0f, 0.0f, collideWallMultiplier * v.z), ForceMode.VelocityChange);
                rbody.velocity = new Vector3(v.x, v.y, collideWallMultiplier * v.z);
            }
        }
    }

    public override void OnEpisodeBegin()
    {
        rbody.velocity = new Vector3(Random.value * 20 - 10, 0.0f, Random.value * 20 - 10);
        var localOrigin = rbody.position - transform.localPosition;
        rbody.position = localOrigin + new Vector3(Random.value * 98.0f - 49.0f, 0.0f, Random.value * 98.0f - 49.0f);
        target.localPosition = new Vector3(Random.value * 100 - 50, 0.5f, Random.value * 100 - 50);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        var targetPos = target.localPosition;
        sensor.AddObservation(targetPos.x);
        sensor.AddObservation(targetPos.z);

        var pos = this.transform.localPosition;
        sensor.AddObservation(pos.x);
        sensor.AddObservation(pos.z);

        var v = rbody.velocity;
        sensor.AddObservation(v.x);
        sensor.AddObservation(v.z);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var discreteAction = actions.DiscreteActions;
        const float force = 5.0f;
        float forceX = force, forceZ = force;
        if (discreteAction[0] == 1)
        {
            forceX = -force;
        }

        if (discreteAction[1] == 1)
        {
            forceZ = -force;
        }

        var actionForce = new Vector3(forceX, 0.0f, forceZ);
        rbody.AddForce(actionForce);

        var distance = Vector3.Distance(this.transform.localPosition, target.localPosition);
        if (distance > 1.0f)
        {
            SetReward(-distance);
        }
        else
        {
            EndEpisode();
        }
    }
}