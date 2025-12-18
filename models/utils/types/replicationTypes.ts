/////////////////// Types ////////////////////
export type ReplicationDocument = {
  _id?: string;
  source: string;
  target: string;
  continuous: boolean;
  create_target: boolean;
  selector?: object;
  transforms?: object;
  filter?: string;
};

///////////////// Functions ////////////////////

export function createReplicationDocument(
  source: string,
  target: string,
  continuous: boolean = true,
  create_target: boolean = true,
  selector?: object,
): ReplicationDocument {
  const replicationDoc: ReplicationDocument = {
    source: source,
    target: target,
    continuous,
    create_target,
  };

  // Only add selector if provided, and always set filter
  if (selector) {
    // replicationDoc.filter = "_selector";
    replicationDoc.selector = selector;
  }

  return replicationDoc;
} 

// export function createReplicationDocument(
//   source: string,
//   target: string,
//   continuous: boolean = true,
//   create_target: boolean = true,
//   selector?: object,
//   transforms?: object
// ) {
//   const replicationDoc: ReplicationDocument = {
//     _id: `replication_${source}_to_${target}`,
//     source,
//     target,
//     continuous,
//     create_target,
//     selector,
//     transforms,
//   };
//   return replicationDoc;
// }

// export function createTransformations(
//   value: string = CouchManager.defaultValues.nameOfAgent,
//   field: string = "_meta.source",
//   operation: string = "set"
// ) {
//   return {
//     operation,
//     field,
//     value,
//   };
// }

export function defaultSelector() {
  return {
    _id: { $gt: null },
  };
}
