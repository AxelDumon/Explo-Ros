import rclnodejs from "rclnodejs";
import { CouchManager } from "../../db/CouchManager.js";
import { Document } from "./general.js";

//////////////// Types ////////////////////

export type Agent = {
  x: number;
  y: number;
};

export type AgentDocument = Document &
  Agent & { type: "agent"; meta?: { source: string; created_at: string } };

/////////////// Functions ////////////////////

//// Type guards

export function isPoint(obj: any): obj is rclnodejs.geometry_msgs.msg.Point {
  return (
    obj != null &&
    (typeof obj.x).toLowerCase() === "number" &&
    (typeof obj.y).toLowerCase() === "number" &&
    (typeof obj.z).toLowerCase() === "number"
  );
}

//// Converters

export function fromPointToAgentDocument(
  pose: rclnodejs.geometry_msgs.msg.Point,
  nameOfAgent: string = CouchManager.defaultValues.nameOfAgent
): AgentDocument {
  return {
    _id: nameOfAgent,
    type: "agent",
    x: pose.x,
    y: pose.y,
  } as AgentDocument;
}
