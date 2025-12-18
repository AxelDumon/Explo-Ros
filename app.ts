import rclnodejs from "rclnodejs";
import { CouchManager } from "./models/db/CouchManager.js";
import { Agent } from "./models/exploration/Agents.js";
import { World } from "./models/exploration/World.js";

async function launchCouchManagerNode() {
  try {
    console.log("Creating CouchManager node...");
    const couchManager = await CouchManager.CouchManagerRosFactory(
      "./models/db/couchManagerConf.json"
    );

    console.log("Starting CouchManager ROS node...");
    couchManager.spin();
  } catch (error) {
    console.error("CouchManager error:", error);
    process.exit(1);
  }
}

async function prepareRos() {
  try {
    console.log("Initializing rclnodejs...");
    await rclnodejs.init();
    console.log("rclnodejs initialized successfully!");

    process.on("SIGINT", async () => {
      console.log("\nShutting down gracefully...");
      rclnodejs.shutdown();
      process.exit(0);
    });

    process.on("SIGTERM", async () => {
      console.log("\nShutting down gracefully...");
      rclnodejs.shutdown();
      process.exit(0);
    });
  } catch (error) {
    console.error("Error during initialization:", error);
    process.exit(1);
  }
}

async function launchAgentNode() {
  try {
    console.log("Creating Agent node...");
    const agent = new Agent("agent01", World.defaultMap().cells.length);
    agent.explore(200, World.defaultMap()).catch((error) => {
      console.error("Exploration error:", error);
    });
    console.log("Starting Agent ROS node...");
    agent.spin();
  } catch (error) {
    console.error("Agent error:", error);
    process.exit(1);
  }
}

async function main() {
  await prepareRos();
  launchCouchManagerNode();
  await new Promise((resolve) => setTimeout(resolve, 2000)); // wait for CouchManager
  launchAgentNode();
}

main();
