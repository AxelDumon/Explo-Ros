import rclnodejs from "rclnodejs";

import { World } from "./World.js";
import { Cell, isMapUpdate, MapUpdate } from "../utils/types/cellTypes.js";
import { wrongTypeLog } from "../utils/types/general.js";

export class Agent extends rclnodejs.Node {
  static node_name = "agent_node";
  // Instance properties
  nameOfAgent: string;
  isExploring: boolean;
  position: Cell;
  map: Uint8Array;
  // ROS
  posPublisher: rclnodejs.Publisher<any>;
  mapPublisher: rclnodejs.Publisher<any>;
  replicationSubscriber: rclnodejs.Subscription;

  constructor(name: string, worldSize: number) {
    super(Agent.node_name, `/${name}`);
    this.nameOfAgent = name;
    this.position = { x: 0, y: 0, value: 50 };
    this.isExploring = false;
    // ROS
    const mapUpdateType = rclnodejs.require("node_pkg/msg/MapUpdate") as any;
    this.posPublisher = this.createPublisher(
      "geometry_msgs/msg/Point",
      "update_pos"
    );
    this.mapPublisher = this.createPublisher(mapUpdateType, "update_map");
    this.replicationSubscriber = this.createSubscription(
      mapUpdateType,
      "replication_changes",
      this.replicationSubCallback.bind(this)
    );
    this.map = new Uint8Array(worldSize).fill(50);
  }

  ////// Instance methods //////

  // ROS Callbacks

  replicationSubCallback(msg: any): void {
    if (isMapUpdate(msg)) this.updateLocalMap(msg);
    else wrongTypeLog(this.replicationSubCallback.name, msg);
  }

  updateLocalMap(mapUpdate: MapUpdate): void {
    mapUpdate.cells.forEach((cell) => {
      const index = cell.y * Math.sqrt(this.map.length) + cell.x;
      this.map[index] = cell.value;
    });
  }

  // Exploration method

  async explore(DELAY = 1000, world: World): Promise<void> {
    if (this.isExploring) throw new Error("Agent is already exploring");
    this.isExploring = true;
    console.log(
      `[${this.explore.name}] Agent ${this.nameOfAgent} started exploring`
    );
    // const mapUpdateType = rclnodejs.require("node_pkg/msg/MapUpdate") as any;

    const startPos = world.getRandomEmptyCell();
    if (!startPos) {
      console.log(
        `[${this.explore.name}] No undiscovered cells left / No empty cells available (Start)`
      );
      this.isExploring = false;
      return;
    }
    this.updatePosition(startPos);
    while (true) {
      let foundFrontier = false;

      // Array like [{x,y,value=50}, {x,y,value=50}, ...]
      const undiscoveredNeighbors = this.getUndiscoveredNeighbors(
        this.position.x,
        this.position.y
      );

      if (undiscoveredNeighbors && undiscoveredNeighbors.length > 0) {
        // Reveal and save
        const revealedCells = world.revealCells(undiscoveredNeighbors);
        this.updateLocalMap({ cells: revealedCells });
        this.mapPublisher.publish({ cells: revealedCells });

        // Look for cells to go to
        const emptyCells = revealedCells.filter((cell) => cell.value === 0);
        if (emptyCells.length === 0) {
          console.log(
            `[${this.explore.name}] No empty undiscovered neighbors for Agent ${this.nameOfAgent}`
          );
          await new Promise((resolve) => setTimeout(resolve, DELAY));
          continue;
        }
        const targetCell =
          emptyCells[Math.floor(Math.random() * emptyCells.length)];
        this.updatePosition(targetCell);
        `[${this.explore.name}] Agent ${this.nameOfAgent} explored (${targetCell.x}, ${targetCell.y})`;
        foundFrontier = true;
        await new Promise((resolve) => setTimeout(resolve, DELAY));
      }
      if (!foundFrontier) {
        console.log(
          `[${this.explore.name}] No frontier found for Agent ${this.nameOfAgent}, teleporting...`
        );
        const undiscoveredCell = this.getRandomUndiscoveredCell();
        if (!undiscoveredCell) {
          console.log(
            `[${this.explore.name}] No undiscovered cells left, exploration finished`
          );
          break;
        }
        const revealedCell = world.revealCells([undiscoveredCell]);
        this.updateLocalMap({ cells: revealedCell });
        this.mapPublisher.publish({ cells: revealedCell });
        if (revealedCell[0].value === 0) {
          this.updatePosition(revealedCell[0]);
          console.log(
            `[${this.explore.name}] Agent ${this.nameOfAgent} teleported to (${revealedCell[0].x}, ${revealedCell[0].y})`
          );
        }
        await new Promise((resolve) => setTimeout(resolve, DELAY));
      }
    }

    this.isExploring = false;
    console.log(
      `[${this.explore.name}] Agent ${this.nameOfAgent} finished exploring`
    );
  }

  getUndiscoveredNeighbors(x: number, y: number): Cell[] {
    const neighbors: Cell[] = [];
    const directions = [
      { dx: -1, dy: 0 },
      { dx: 1, dy: 0 },
      { dx: 0, dy: -1 },
      { dx: 0, dy: 1 },
    ];
    for (const dir of directions) {
      const nx = x + dir.dx;
      const ny = y + dir.dy;
      if (
        nx >= 0 &&
        nx < Math.sqrt(this.map.length) &&
        ny >= 0 &&
        ny < Math.sqrt(this.map.length) &&
        this.map[ny * Math.sqrt(this.map.length) + nx] === 50
      ) {
        neighbors.push({ x: nx, y: ny, value: 50 });
      }
    }
    return neighbors;
  }

  getRandomUndiscoveredCell(): Cell | null {
    const undiscoveredCells: { x: number; y: number; value: 50 }[] = [];
    for (let y = 0; y < Math.sqrt(this.map.length); y++) {
      for (let x = 0; x < Math.sqrt(this.map.length); x++) {
        const index = y * Math.sqrt(this.map.length) + x;
        if (this.map[index] === 50) {
          undiscoveredCells.push({ x, y, value: 50 });
        }
      }
    }
    if (undiscoveredCells.length === 0) {
      return null;
    }
    return undiscoveredCells[
      Math.floor(Math.random() * undiscoveredCells.length)
    ];
  }

  updatePosition({ x, y }: Cell) {
    const poseObj = rclnodejs.createMessageObject("geometry_msgs/msg/Point");
    poseObj.x = x;
    poseObj.y = y;
    poseObj.z = 0;
    this.position.x = x;
    this.position.y = y;
    this.posPublisher.publish(poseObj);
  }
}
