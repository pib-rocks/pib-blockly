import * as Blockly from "blockly";

// export const moveToPose =
//     Blockly.common.createBlockDefinitionsFromJsonArray([
//         {
//             type: "move_to_pose",
//             message0: "move to %1",
//             args0: [
//                 {
//                     type: "field_dropdown",
//                     name: "POSENAME",
//                 },
//             ],
//             previousStatement: null,
//             nextStatement: null,
//             colour: 355,
//             tooltip: "moves to selected pose",
//             helpUrl: "",
//         },
//     ]);



export const moveToPose = Blockly.Blocks["move_to_pose"] = {
      init(){
        const input = this.appendDummyInput()
          .appendField("move to")
          .appendField(new Blockly.FieldDropdown(() => this.getPoses()), "POSENAME");
          this.setColour(355);
          this.setPreviousStatement(true, null);
          this.setNextStatement(true, null);
          this.setTooltip("moves to selected pose");
          this.setHelpUrl("");
      },

      getPoses(){
        // Initial placeholder before options are fetched
        return [['Loading...', 'LOADING']];
      }
};