import { DesignDoc, Document, wrongTypeLog } from "../utils/types/general.js";
import {
  isPoint,
  fromPointToAgentDocument,
} from "../utils/types/agentTypes.js";
import {
  // fromUint8ArrayToCellDocumentArray,
  // fromCellDocumentToUint8,
  fromMapUpdateToCellDocumentArray,
  fromCellDocumentToCell,
  MapUpdate,
  isMapUpdate,
} from "../utils/types/cellTypes.js";
import rclnodejs from "rclnodejs";
import fs from "fs";
import designDocs from "./views/index.js";
import { CouchChangeListener } from "./CouchChangeListener.js";
import {
  createReplicationDocument,
  defaultSelector,
  ReplicationDocument,
} from "../utils/types/replicationTypes.js";

export interface CouchManagerConfig {
  dbName: string;
  serverUrl: string;
  adminUser: { user: string; password: string };
  nameOfAgent: string;
}

export class CouchManager extends rclnodejs.Node {
  //// Static variables ////
  static contentTypeJson = { "Content-Type": "application/json" };
  static defaultValues: CouchManagerConfig = {
    dbName: "v2grid",
    serverUrl: "http://localhost:5984",
    adminUser: { user: "admin", password: "password" },
    nameOfAgent: "default_agent",
  };
  //// Instance variables ////
  dbName: string = CouchManager.defaultValues.dbName;
  serverUrl: string = CouchManager.defaultValues.serverUrl;
  dbUrl: string = `${this.serverUrl}/${this.dbName}`;
  authHeader: string = `Basic ${Buffer.from(
    `${process.env.COUCHDB_USER || CouchManager.defaultValues.adminUser.user}:${
      process.env.COUCHDB_PASSWORD ||
      CouchManager.defaultValues.adminUser.password
    }`
  ).toString("base64")}`;
  // ROS Subscribers and Publishers
  mapSubscriber?: rclnodejs.Subscription;
  posSubscriber?: rclnodejs.Subscription;
  replPublisher?: rclnodejs.Publisher<any>;
  nameOfAgent: string = CouchManager.defaultValues.nameOfAgent;
  changeListener?: CouchChangeListener;
  /// Constructor

  constructor(name: string = "couch_manager", namespace: string = "") {
    super(name, namespace);
  }

  ///// Static Methods //////

  static async CouchManagerRosFactory(
    jsonConfigPath: string
  ): Promise<CouchManager> {
    console.log("CouchManagerRos Factory");
    const config = await CouchManager.getConfigurationFromFile(jsonConfigPath);
    const couchManager = new CouchManager(
      `couch_manager`,
      `/${config.nameOfAgent}`
    );
    // await CouchManager.configureFromFile(jsonConfigPath, couchManager);
    CouchManager.configureFromConfig(config, couchManager);
    await couchManager.initBase();
    await couchManager.createReplicators("./models/db/couchHosts.json");
    const mapUpdateType = rclnodejs.require("node_pkg/msg/MapUpdate") as any;
    couchManager.replPublisher = couchManager.createPublisher(
      mapUpdateType,
      "replication_changes"
    );
    couchManager.mapSubscriber = couchManager.createSubscription(
      mapUpdateType,
      "update_map",
      couchManager.mapSubCallback.bind(couchManager)
    );
    couchManager.posSubscriber = couchManager.createSubscription(
      "geometry_msgs/msg/Point",
      "update_pos",
      couchManager.posSubCallback.bind(couchManager)
    );
    couchManager.changeListener = new CouchChangeListener({
      dbManager: couchManager,
      since: "now",
      onChange: couchManager.createOnChangeHandler.bind(couchManager),
    });
    couchManager.changeListener.start().catch((error) => {
      console.error("CouchDB Change Listener error:", error);
    });

    return couchManager;
  }

  ////// Instance Methods //////

  //// Callbacks

  /**
   * Callback for position updates from ROS.
   * @param msg - ROS message containing the agent position
   */
  async posSubCallback(msg: any): Promise<void> {
    if (isPoint(msg)) {
      const doc = fromPointToAgentDocument(msg, this.nameOfAgent);

      doc.meta = {
        source: this.nameOfAgent,
        created_at: new Date().toISOString(),
      };

      await this.saveDocsInBatches([doc]);
    } else wrongTypeLog(this.posSubCallback.name, msg);
  }

  /**
   * Callback for map updates from ROS.
   * @param {any} msg - ROS message containing the map data
   * @returns {Promise<void>}
   */
  async mapSubCallback(msg: any): Promise<void> {
    if (isMapUpdate(msg)) {
      const cellDocs = fromMapUpdateToCellDocumentArray(msg);
      cellDocs.forEach((doc) => {
        doc.meta = {
          source: this.nameOfAgent,
          created_at: new Date().toISOString(),
        };
      });
      await this.saveDocsInBatches(cellDocs);
    }
    // await this.saveDocsInBatches(fromUint8ArrayToCellDocumentArray(msg.data));
    else wrongTypeLog(this.mapSubCallback.name, msg);
  }

  //// CouchDB Methods

  /**
   * Attach revisions to documents IDs.
   * @param {{ id : string }[]} ids - List of documents IDs to check
   * @returns {Promise<Record<string, string>>} Results from CouchDB _bulk_get
   */
  async fetchRevs(ids: { id: string }[]): Promise<Record<string, string>> {
    if (ids.length > 0) {
      try {
        const bulkGetRes = await fetch(`${this.dbUrl}/_bulk_get`, {
          method: "POST",
          headers: this.prepareHeader(CouchManager.contentTypeJson),
          body: JSON.stringify({ docs: ids }),
        });

        if (!bulkGetRes.ok) {
          throw new Error(`_bulk_get error: ${bulkGetRes.status}`);
        }
        const bulkGetJson = await bulkGetRes.json();
        const revMap: Record<string, string> = {};
        bulkGetJson.results.forEach(
          (r: { id: string; docs: Array<{ ok: { _rev: string } }> }) => {
            if (r.docs && r.docs[0] && r.docs[0].ok) {
              revMap[r.id] = r.docs[0].ok._rev;
            }
          }
        );
        return revMap;
      } catch (error) {
        console.error("Error fetching revisions:", error);
        return {};
      }
    } else {
      console.log("No IDs provided for fetching revisions.");
      return {};
    }
  }

  /**
   * Post the documents batch to CouchDB.
   * @param {Document[]} batch - List of documents to save
   * @returns {Promise<Array>} CouchDB responses concatenated
   */
  async processBatch(batch: Document[]): Promise<any> {
    const idsToFetch = batch
      .filter((d) => d._id && !d._rev)
      .map((d) => ({ id: d._id }));

    const revMap = await this.fetchRevs(idsToFetch);

    batch.forEach((doc) => {
      if (doc._id && !doc._rev && revMap[doc._id]) {
        doc._rev = revMap[doc._id];
      }
    });

    const res = await fetch(`${this.dbUrl}/_bulk_docs`, {
      method: "POST",
      headers: this.prepareHeader(CouchManager.contentTypeJson),
      body: JSON.stringify({ docs: batch }),
    });

    if (!res.ok) {
      throw new Error(`Bulk docs error: ${res.status} - ${await res.text()}`);
    }

    return res.json();
  }

  /**
   * Save a documents list in CouchDB per batches of 500 docs.
   * @param {Document[]} docs - List of the documents to save
   * @returns {Promise<Array>} CouchDB responses concatenated
   */
  async saveDocsInBatches(docs: Document[]): Promise<Array<Document>> {
    console.log(`[saveDocsInBatches] Saving ${docs.length} documents:`, docs);
    const BATCH_SIZE = 500;

    const batches = [];
    for (let i = 0; i < docs.length; i += BATCH_SIZE) {
      batches.push(docs.slice(i, i + BATCH_SIZE));
    }

    const results = await Promise.all(
      batches.map((batch) => this.processBatch(batch))
    );
    return results;
  }

  static configureFromConfig(
    config: CouchManagerConfig,
    couchManager: CouchManager
  ): CouchManager {
    couchManager.dbName = config.dbName || CouchManager.defaultValues.dbName;
    couchManager.serverUrl =
      config.serverUrl || CouchManager.defaultValues.serverUrl;
    couchManager.dbUrl = `${
      config.serverUrl || CouchManager.defaultValues.serverUrl
    }/${couchManager.dbName}`;
    couchManager.authHeader = `Basic ${Buffer.from(
      `${config.adminUser?.user || CouchManager.defaultValues.adminUser.user}:${
        config.adminUser?.password ||
        CouchManager.defaultValues.adminUser.password
      }`
    ).toString("base64")}`;
    couchManager.nameOfAgent =
      config.nameOfAgent || CouchManager.defaultValues.nameOfAgent;
    console.log("CouchManagerRos configured with:", config);
    return couchManager;
  }

  static async configureFromFile(
    jsonConfigPath: string,
    couchManager: CouchManager
  ): Promise<CouchManager> {
    try {
      fs.readFile(jsonConfigPath, "utf8", (err, jsonStr) => {
        if (err) {
          console.error("Error reading JSON config file:", err);
          return;
        }
        try {
          const config: CouchManagerConfig = JSON.parse(jsonStr);

          couchManager.dbName =
            config.dbName || CouchManager.defaultValues.dbName;
          couchManager.serverUrl =
            config.serverUrl || CouchManager.defaultValues.serverUrl;
          couchManager.dbUrl = `${couchManager.serverUrl}/${couchManager.dbName}`;
          couchManager.authHeader = `Basic ${Buffer.from(
            `${
              config.adminUser?.user ||
              CouchManager.defaultValues.adminUser.user
            }:${
              config.adminUser?.password ||
              CouchManager.defaultValues.adminUser.password
            }`
          ).toString("base64")}`;
          couchManager.nameOfAgent =
            config.nameOfAgent || CouchManager.defaultValues.nameOfAgent;
          console.log("CouchManagerRos configured with:", config);
        } catch (parseErr) {
          console.error("Error parsing JSON config file:", parseErr);
        }
      });
      return couchManager;
    } catch (error) {
      console.error("Error configuring CouchManager from file:", error);
      return couchManager;
    }
  }

  static async getConfigurationFromFile(
    jsonConfigPath: string
  ): Promise<CouchManagerConfig> {
    try {
      const jsonStr = await fs.promises.readFile(jsonConfigPath, "utf8");
      try {
        const config: CouchManagerConfig = JSON.parse(jsonStr);

        config.dbName = config.dbName || CouchManager.defaultValues.dbName;
        config.serverUrl =
          config.serverUrl || CouchManager.defaultValues.serverUrl;
        config.adminUser = {
          user:
            config.adminUser?.user || CouchManager.defaultValues.adminUser.user,
          password:
            config.adminUser?.password ||
            CouchManager.defaultValues.adminUser.password,
        };
        config.nameOfAgent =
          config.nameOfAgent || CouchManager.defaultValues.nameOfAgent;
        console.log("configuration found:", config);

        return config;
      } catch (parseErr) {
        console.error("Error parsing JSON config file:", parseErr);
        return CouchManager.defaultValues;
      }
    } catch (error) {
      console.error("Error reading JSON config file:", error);
      return CouchManager.defaultValues;
    }
  }

  /**
   * Import all design documents from the designDocs repertory into CouchDB.
   * @returns {Promise<void>}
   */
  async importAllDesignDocs(): Promise<void> {
    for (const [name, designDoc] of Object.entries(designDocs)) {
      console.log(`Uploading design document: ${name}`);
      await this.uploadDesignDoc(designDoc);
    }
  }

  /**
   * Initialize CouchDB: create the database and import design documents.
   * @returns {Promise<boolean>} True if initialization succeeded, false otherwise
   */
  async initBase(): Promise<boolean> {
    try {
      await this.createDB();
      // await this.importAllDesignDocs();
      console.log("CouchDB initialized successfully.");
      return true;
    } catch (error) {
      console.error("Error initializing CouchDB:", error);
      return false;
    }
  }

  /**
   * Create the _replicator db if it doesn't exist and post the replication documents to each hosts
   * @param pathToHosts Path to the JSON file containing the hosts configuration
   * @returns
   */
  async createReplicators(pathToHosts: string): Promise<void> {
    try {
      console.log("Creating replicators from hosts file:", pathToHosts);

      const jsonStr = await fs.promises.readFile(pathToHosts, "utf8");
      const hostsDBUrls: Array<string> = JSON.parse(jsonStr);
      const replDocs: ReplicationDocument[] = [];
      for (const hostUrl of hostsDBUrls) {
        if (hostUrl !== this.dbUrl) {
          replDocs.push(
            createReplicationDocument(
              this.dbUrl,
              hostUrl,
              true,
              true,
              defaultSelector(),
              this.authHeader
            )
          );
        }
      }

      if (replDocs.length === 0) {
        console.log("No replication documents to create.");
        return;
      }

      const replicatorDbUrl = `${this.serverUrl}/_replicator`;

      console.log("Ensuring _replicator database exists...");
      const createReplicatorDbRes = await fetch(replicatorDbUrl, {
        method: "PUT",
        headers: this.prepareHeader(),
      });

      if (createReplicatorDbRes.status === 201) {
        console.log("_replicator database created");
      } else if (createReplicatorDbRes.status === 412) {
        console.log("_replicator database already exists");
      } else if (!createReplicatorDbRes.ok) {
        console.error(
          `Failed to create _replicator database: ${
            createReplicatorDbRes.status
          } - ${await createReplicatorDbRes.text()}`
        );
        return;
      }

      await new Promise((resolve) => setTimeout(resolve, 500));
      for (const doc of replDocs) {
        try {
          const existingDocRes = await fetch(
            `${replicatorDbUrl}/${encodeURIComponent(doc._id)}`,
            {
              method: "GET",
              headers: this.prepareHeader(),
            }
          );

          if (existingDocRes.ok) {
            console.log(
              `Replication document for target ${doc.target.toString()} already exists, skipping...`
            );
            continue;
          }

          const response = await fetch(`${replicatorDbUrl}`, {
            method: "POST",
            headers: this.prepareHeader(CouchManager.contentTypeJson),
            body: JSON.stringify(doc),
          });

          if (!response.ok) {
            const errorText = await response.text();
            console.error(
              `Failed to create replication document for target ${doc.target}: ${response.status} - ${errorText}`
            );
          } else {
            console.log(
              `Replication document for target ${doc.target} created successfully`
            );
          }
        } catch (error) {
          console.error(
            `Error creating replication document for target ${doc.target}:`,
            error
          );
        }
      }
      console.log("Replication documents creation completed.");
    } catch (error) {
      console.error("Error parsing CouchDB hosts config file:", error);
    }
  }

  /**
   * Create the CouchDB database if it doesn't exist. If it exists, clear all documents.
   * @returns {Promise<void>}
   */
  async createDB(): Promise<void> {
    const url = this.dbUrl;
    try {
      const response = await fetch(url, {
        method: "PUT",
        headers: { Authorization: this.authHeader },
      });

      if (response.status === 201) {
        console.log(`Database created: ${this.dbName}`);
      } else if (response.status === 412) {
        console.log(`Database already exists: ${this.dbName}`);
        // await this.purgeAllNonDesignDocs();
      } else {
        throw new Error(`Failed to create database: ${response.statusText}`);
      }
    } catch (error) {
      console.error(`Error creating database:`, error);
    }
  }

  async purgeAllNonDesignDocs(): Promise<void> {
    console.log("Clearing documents for fresh start...");
    try {
      const allDocsResponse = await fetch(
        `${this.dbUrl}/_all_docs?conflicts=true`,
        {
          headers: this.prepareHeader(),
        }
      );
      const allDocsData = await allDocsResponse.json();
      if (allDocsData == undefined || allDocsData.rows == undefined) {
        console.log("No documents found to delete.");
        return;
      }

      const purgeRequest: Record<string, string[]> = {};
      allDocsData.rows.forEach((row: any) => {
        if (!row.id.startsWith("_design")) {
          purgeRequest[row.id] = [row.value.rev];
        }
      });

      if (Object.keys(purgeRequest).length === 0) {
        console.log("[CouchManager] No documents to purge");
        return;
      }

      const purgeRes = await fetch(`${this.dbUrl}/_purge`, {
        method: "POST",
        headers: this.prepareHeader(CouchManager.contentTypeJson),
        body: JSON.stringify(purgeRequest),
      });

      const purgeResult = await purgeRes.json();
      console.log(
        `[CouchManager] Purged ${
          Object.keys(purgeRequest).length
        } documents completely`
      );
      console.log("[CouchManager] Purge result:", purgeResult);
    } catch (error) {
      console.error("Error purging documents:", error);
    }
  }

  /**
   * Upload a design document (views, update handlers, etc.) to CouchDB.
   * @param designDoc - The design document to upload
   */
  async uploadDesignDoc(designDoc: DesignDoc): Promise<void> {
    console.log("Uploading design document:", designDoc._id);
    const url = `${this.dbUrl}/${designDoc._id}`;
    console.log("Design document URL:", url);
    try {
      // Check if the design document already exists
      const existingDoc = await fetch(url, {
        method: "GET",
        headers: { Authorization: this.authHeader },
      });

      if (existingDoc.ok) {
        const existingData = await existingDoc.json();
        designDoc._rev = existingData._rev; // Add the revision ID to update the document
      }

      // Upload the design document
      const response = await fetch(url, {
        method: "PUT",
        headers: this.prepareHeader(CouchManager.contentTypeJson),
        body: JSON.stringify(designDoc),
      });

      if (!response.ok) {
        throw new Error(
          `Failed to upload design document: ${response.statusText}`
        );
      }

      console.log(`Design document uploaded: ${designDoc._id}`);
    } catch (error) {
      console.error("Error uploading design document:", error);
    }
  }

  //// Utilities

  /**
   * Prepare headers for CouchDB requests.
   * @param args Arguments to add to the headers
   * @returns {Record<string, string>} Prepared headers
   */
  prepareHeader(...args: Record<string, any>[]): Record<string, string> {
    const headers: Record<string, string> = {
      Authorization: this.authHeader,
    };
    args.forEach((arg) => {
      for (const [key, value] of Object.entries(arg)) {
        headers[key] = value;
      }
    });
    return headers;
  }

  createOnChangeHandler(change: any) {
    const doc = change.doc;
    // console.log("Change detected:", doc);
    if (doc && doc.meta && doc.meta.source) {
      if(doc.meta.source === this.nameOfAgent) {
        return;
      }
      this.replPublisher?.publish({
        cells: [fromCellDocumentToCell(doc)],
      } as MapUpdate);
    }
  }
}
