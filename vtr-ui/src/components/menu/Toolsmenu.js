import React from "react";

import Box from "@material-ui/core/Box";
import Button from "@material-ui/core/Button";
import CheckIcon from "@material-ui/icons/Check";
import AndroidIcon from "@material-ui/icons/Android";
import TimelineIcon from "@material-ui/icons/Timeline";
import CancelIcon from "@material-ui/icons/Cancel";
import { withStyles } from "@material-ui/core/styles";

const styles = (theme) => ({});

class ToolsMenu extends React.Component {
  constructor(props) {
    super(props);

    this.state = {};
  }

  render() {
    const { mode, toolsState, selectTool, requireConf } = this.props;
    return (
      <>
        <Box
          // Positions
          position={"absolute"}
          top={0}
          right={0}
          zIndex={1000}
          // Flexbox
          display={"flex"}
          flexDirection={"column"}
          // Spacing
          m={0.25}
        >
          {mode === "vtr" && (
            <>
              <Box m={0.25} width={150}>
                <Button
                  color={toolsState.moveRobot ? "secondary" : "primary"}
                  disableElevation={true}
                  fullWidth={true}
                  startIcon={<AndroidIcon />}
                  variant={"contained"}
                  onClick={() => selectTool("moveRobot")}
                >
                  Move Robot
                </Button>
              </Box>
              <Box m={0.25} width={150}>
                <Button
                  color={toolsState.moveMap ? "secondary" : "primary"}
                  disableElevation={true}
                  fullWidth={true}
                  startIcon={<TimelineIcon />}
                  variant={"contained"}
                  onClick={() => selectTool("moveMap")}
                >
                  Move Graph
                </Button>
              </Box>
              <Box m={0.25} width={150}>
                <Button
                  color={toolsState.pinGraph ? "secondary" : "primary"}
                  disableElevation={true}
                  fullWidth={true}
                  startIcon={<CancelIcon />}
                  variant={"contained"}
                  onClick={() => selectTool("pinGraph")}
                >
                  Pin Graph
                </Button>
              </Box>
            </>
          )}
        </Box>
        <Box
          // Positions
          position={"absolute"}
          top={0}
          right={155}
          zIndex={1000}
          // Flexbox
          display={"flex"}
          flexDirection={"column"}
          // Spacing
          m={0.25}
        >
          {(toolsState.moveMap ||
            toolsState.moveRobot ||
            toolsState.pinGraph) && (
            <Box m={0.25} width={150}>
              <Button
                color="secondary"
                disableElevation={true}
                fullWidth={true}
                startIcon={<CheckIcon />}
                variant={"contained"}
                onClick={() => requireConf()}
              >
                Confirm
              </Button>
            </Box>
          )}
        </Box>
      </>
    );
  }
}

export default withStyles(styles)(ToolsMenu);
