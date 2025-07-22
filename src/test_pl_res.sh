GRIPPER_OBJ=(
  "025_mug"
  "017_orange"
  "013_apple"
  "011_banana"
  "017_orange"
  "006_mustard_bottle"
  "010_potted_meat_can"
  "024_bowl"
  "006_mustard_bottle"
  "011_banana"
  "025_mug"
  "013_apple"
  "017_orange"
)




COMMANDS=(

  "Place the mug to the left of the fruits."
  "Place the orange to the right of the bowl."
  "Place the apple to the left of the condiments."
  "Place the banana on top of the plate."
  "Place the orange inside the bowl."
  "Place the mustard behind the apple."
  "Place the canned meat behind the other objects."
  "Place the bowl in front of the orange."
  "Place the mustard in front of the dishes."
  "Place the banana near the apple."
  "Place the mug near the fruits."
  "Place the apple next to the mustard."
  "Place the orange next to the other objects."
  
)


#python test_placing_reasoner.py "${GRIPPER_OBJ[12]}" "${COMMANDS[12]}" --rep 100 --output_file test_output/scenario13.json 


# python test_placing_reasoner.py "011_banana" "place the banana on the table" --output_file test_output/scenario14.json  --rep 100 || true
# python test_placing_reasoner.py "011_banana" "place the banana on top of the plate" --output_file test_output/scenario15.json  --rep 100 || true
python test_placing_reasoner.py "011_banana" "release the banana" --output_file test_output/scenario16.json --rep 100 || true