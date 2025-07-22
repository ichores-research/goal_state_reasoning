import subprocess

from concurrent.futures import ThreadPoolExecutor, as_completed


commands = {
    1: 'pick up this',
    5: 'pick up the yellow object',
    6: 'pick up a yellow banana',
    7: 'pick up an object similar to banana in color',
    9: ' pick up an object right to the banana',
    10: 'pick up an object left to the red object',
    21: 'move the apple behind the yellow object',
    23: 'first pick up an object similar to banana in color, then place it right to the apple',
    24: 'first pick up an apple, then place it in front of the yellow object',
    25: 'first move banana, then pick up a red object',
    26: 'first move the apple left to the mustard then pick up a banana',
    29: 'place the object right of banana but first pick up an apple',
    30: 'move the banana to the left, but first move the apple right to the banana'

}

scenes ={
    1: 'banana',
    5: 'banana, 2 red apples',
    6: 'banana mustard apple',
    7: 'banana, on the right: apple, on the left of banana: mustard',
    9: 'banana, on the right: apple, on the left of banana: mustard',
    10: 'apple, on the right: banana, on the left of apple: mustard',
    21: 'apple in front of mustard',
    23: 'apple, to the left: lemon',
    24: 'apple behind lemon',
    25: 'banana, apple',
    26: 'apple in front of mustard, banana on the right of apple',
    29: 'mustard left of banana, apple in front of mustard',
    30: 'banana left of apple'
}



num_reps = 5


def run_command(command_number, rep_index, command):
    print(f"Running command {command_number}, reimport subprocesspetition {rep_index + 1}/{num_reps}")
    result = subprocess.run(
        ["python", "agent.py", command],
        shell=True, capture_output=True, text=True
    )
    output_file = f"test_output/michal_{command_number}_{rep_index}"
    with open(output_file, "w") as file:
        file.write(result.stdout)
    print(f"Output saved to {output_file}")
    return output_file

if __name__ == "__main__":
    start_from = int(input("Start from command number: "))

    for command_number in commands.keys():
        if command_number < start_from:
            continue

        print(f"\n\nCommand {command_number}: {commands[command_number]}")
        print(f"Scene: {scenes[command_number]}")
        input("Press Enter to continue...")

        with ThreadPoolExecutor(max_workers=5) as executor:
            futures = [
                executor.submit(run_command, command_number, i, commands[command_number])
                for i in range(num_reps)
            ]

            # Optional: Wait for all threads to complete
            for future in as_completed(futures):
                future.result()  # Can be used to catch exceptions if needed
