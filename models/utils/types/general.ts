// https://docs.couchdb.org/en/stable/json-structure.html#couchdb-document
//////////////////// Types ////////////////////

export type Document = {
  _id: string;
  _rev?: string;
};

export type DesignDoc = {
  _id: string; // The ID of the design document (e.g., "_design/example")
  _rev?: string; // Optional revision ID for updates
  views: {
    [viewName: string]: {
      map: string; // The map function as a string
      reduce?: string; // Optional reduce function as a string
    };
  };
  language?: string; // Optional language (default is "javascript")
  options?: {
    partitioned?: boolean; // Optional partitioning option
  };
};

// export type AllDocs = {
//   total_rows: number;
//   offset: number;
//   rows: Array<{
//     id: string;
//     key: string;
//     value: {
//       rev: string;
//     };
//     doc?: CellDocument | AgentDocument; // The actual document, if include_docs=true was used
//   }>;
// };

//////////////////// Functions ////////////////////

export function wrongTypeLog(functionName: string, msg: any): void {
  console.error(`[CouchManager] Received data is of wrong type in function ${functionName}.`);
  try {
    console.warn(`Type of data w/obj : ${typeof msg}`);
    console.warn(msg);
  } catch (e) {
    console.error("[CouchManager] Error logging received msg:", e);
  }
}
