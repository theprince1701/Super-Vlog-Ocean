using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Ocean : MonoBehaviour
{
    public static Ocean Instance;
    
    [SerializeField] private Material oceanMaterial;
    
    
    [Serializable]
    struct Wave
    {
        public float amplitude;
        public float steepness;
        public float speed;
        public float frequency;
        public Vector2 direction;

        public Wave(float amplitude, float steepness, float speed, float frequency, Vector2 direction)
        {
            this.amplitude = amplitude;
            this.steepness = steepness;
            this.speed = speed;
            this.frequency = frequency;
            this.direction = direction;
        }
    }

    private List<Wave> waves = new List<Wave>();

    private const int NR_WAVES = 8;

    private void Awake()
    {
        Instance = this;
        InitWaves();
    }

    private void InitWaves()
    {
        for (int i = 0; i < NR_WAVES; i++)
        {
            float amp = oceanMaterial.GetFloat("_Amplitube" + GenerateID(i));
            float steepness = oceanMaterial.GetFloat("_Steepness" + GenerateID(i));
            float frequency = oceanMaterial.GetFloat("_Frequency" + GenerateID(i));
            float speed = oceanMaterial.GetFloat("_Speed" + GenerateID(i));
            Vector2 dir = oceanMaterial.GetVector("_Direction" + GenerateID(i));
            
            Wave wave = new Wave(amp, steepness,speed,frequency, dir);
            
            waves.Add(wave);
        }
    }

    public float SampleHeightAtPosition(Vector3 position)
    {
        float height = 0.0f;

        for (int i = 0; i < waves.Count; i++)
        {
            height += SampleHeight(position, waves[i]);
        }

        return height;
    }

    private float SampleHeight(Vector3 pos, Wave wave)
    {
        Vector2 dir = wave.direction.normalized;
        Vector2 negatedDir = -1 * dir;

        float dot = Vector2.Dot(pos,negatedDir * wave.frequency);
        float time = wave.speed * Time.time;
        float total = dot + time;

        float waveAmp = wave.amplitude * wave.steepness;
        float n = waveAmp * dir.y;
        return Mathf.Cos(total) * n;
    }

    string GenerateID(int i)
    {
        string result = string.Empty;

        if (i != 0)
        {
            result = i.ToString();
        }

        return result;
    }
}
