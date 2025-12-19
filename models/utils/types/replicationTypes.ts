/////////////////// Types ////////////////////
// export type ReplicationDocument = {
//   _id?: string;
//   source: string;
//   target: string;
//   continuous: boolean;
//   create_target: boolean;
//   selector?: object;
//   transforms?: object;
//   filter?: string;
// };

export type ReplicationDocument = {
  _id: string;
  source: string | {
    url: string;
    headers?: Record<string, string>;
  };
  target: string | {
    url: string;
    headers?: Record<string, string>;
  };
  continuous?: boolean;
  create_target?: boolean;
  selector?: object;
  filter?: string;
  user_ctx?: {
    name: string;
    roles: string[];
  };
};

///////////////// Functions ////////////////////

export function createReplicationDocument(
  source: string,
  target: string,
  continuous: boolean = true,
  create_target: boolean = true,
  selector?: object,
  authHeader?: string
): ReplicationDocument {
  const sourceHost = new URL(source).host.replace(/[.:]/g, '_');
  const targetHost = new URL(target).host.replace(/[.:]/g, '_');

  const replicationDoc: ReplicationDocument = {
    _id: `repl_${sourceHost}_to_${targetHost}`,
    source: {
      url: source,
      headers: {
        Authorization: authHeader || "Basic YWRtaW46cGFzc3dvcmQ=" // admin:password base64
      }
    },
    target: {
      url: target,
      headers: {
        Authorization: authHeader || "Basic YWRtaW46cGFzc3dvcmQ="
      }
    },
    // source: source,
    // target: target,
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
