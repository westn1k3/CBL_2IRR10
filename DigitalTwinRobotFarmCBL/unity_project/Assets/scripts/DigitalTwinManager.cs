// DigitalTwinManager.cs
// Visual or animated response to robot instructions

using UnityEngine;

public class DigitalTwinManager : MonoBehaviour
{
    public GameObject waterBot;
    public GameObject pesticideBot;
    public GameObject seedBot;

    public void ReceiveInstruction(string instruction)
    {
        Debug.Log("Received Instruction: " + instruction);

        // Reset all bots' animations first (stop any motion)
        StopAllBots();

        if (instruction.Contains("Water"))
        {
            AnimateBot(waterBot, Vector3.up);
        }
        else if (instruction.Contains("Pesticide"))
        {
            AnimateBot(pesticideBot, Vector3.right);
        }
        else if (instruction.Contains("Seed"))
        {
            AnimateBot(seedBot, Vector3.forward);
        }
    }

    private void AnimateBot(GameObject bot, Vector3 direction)
    {
        bot.transform.Rotate(direction * 45f);  // Simple visual response
    }

    private void StopAllBots()
    {
        waterBot.transform.rotation = Quaternion.identity;
        pesticideBot.transform.rotation = Quaternion.identity;
        seedBot.transform.rotation = Quaternion.identity;
    }
}
