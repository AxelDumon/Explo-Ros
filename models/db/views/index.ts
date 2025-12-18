import fs from "node:fs";
import path from "path";
import { DesignDoc } from "../../utils/types/general.js";
import { fileURLToPath } from "url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

export type DesignDocs = Record<string, DesignDoc>;

const designDocs: DesignDocs = {};

// Path to the design_docs folder
const designDocsPath = path.join(__dirname, "design_docs");

// Read all files in the design_docs folder
async function loadDesigndocs() {
  const files = fs.readdirSync(designDocsPath);
  for (const file of files) {
    if (file.endsWith(".json")) {
      const docName = path.basename(file, ".json"); // Use the file name (without extension) as the key
      const docPath = path.join(designDocsPath, file);
      const designDoc = await import(docPath, { assert: { type: "json" } });

      if (!designDoc.default._id) {
        designDoc.default._id = `_design/${docName}`;
      }

      designDocs[docName] = designDoc.default; // Dynamically require the JSON file
    }
  }
}

await loadDesigndocs();

export default designDocs;
