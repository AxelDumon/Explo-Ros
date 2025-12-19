import { CouchManager } from "./CouchManager";

export class CouchChangeListener {
  static readonly defaultBackoffTime = 1000; // 1 second
  static readonly defaultMaxBackoffTime = 15000; // 15 seconds
  dbManager: CouchManager;
  since: string;
  onChange: (any: any) => void;
  running: boolean;
  backoffTime: number;
  maxBackoffTime: number;
  controller: AbortController;
  constructor({
    dbManager: dbManager,
    since = "now",
    onChange,
  }: {
    dbManager: CouchManager;
    since?: string;
    onChange: (any: any) => void;
  }) {
    this.dbManager = dbManager;
    this.since = since;
    this.onChange = onChange;
    this.running = false;
    this.backoffTime = CouchChangeListener.defaultBackoffTime;
    this.maxBackoffTime = CouchChangeListener.defaultMaxBackoffTime;
    this.controller = new AbortController();
    this.controller.signal.addEventListener("abort", () => {
      this.running = false;
      console.log("[CouchChangeListener] Listener aborted.");
    });
    this.dbManager = dbManager;
  }

  async start() {
    this.running = true;
    console.log(
      `[CouchChangeListener] Starting listener on ${this.dbManager.dbUrl} since=${this.since}`
    );

    while (this.running) {
      try {
        const streamUrl = `${this.dbManager.dbUrl}/_changes?feed=continuous&include_docs=true&since=${this.since}`;
        const response = await fetch(streamUrl, {
          headers: this.dbManager.prepareHeader(),
          signal: this.controller.signal,
        });
        if (!response.ok || !response.body) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }

        this.backoffTime = CouchChangeListener.defaultBackoffTime;

        let buffer = "";
        for await (const chunk of response.body as any) {
          const text = this.normalizeChunk(chunk);
          buffer += text;

          let nl: number;
          while ((nl = buffer.indexOf("\n")) >= 0) {
            const line = buffer.slice(0, nl).trim();
            buffer = buffer.slice(nl + 1);
            if (!line) continue;

            try {
              const change = JSON.parse(line);
              if (change && change.seq) {
                this.since = change.seq;
              }
              if (!change.doc || !change.doc.meta) continue;
              this.onChange(change);
            } catch (e) {
              console.error(
                "[CouchChangeListener] Failed to parse change line:",
                line,
                e
              );
              continue;
            }
          }
        }
      } catch (error) {
        if (!this.running) break;
        console.error("[CouchChangeListener] Error in change listener:", error);
      }
      console.log(
        `[CouchChangeListener] Reconnecting in ${this.backoffTime} ms...`
      );
      await new Promise((resolve) => setTimeout(resolve, this.backoffTime));
      this.backoffTime = Math.min(this.backoffTime * 2, this.maxBackoffTime);
    }
  }

  stop() {
    this.running = false;
    this.controller.abort();
    console.log("[CouchChangeListener] Listener stopped.");
  }

  private normalizeChunk(chunk: any): string {
    if (typeof chunk === "string") return chunk;
    if (chunk instanceof Buffer) return chunk.toString("utf8");
    if (chunk instanceof Uint8Array) return Buffer.from(chunk).toString("utf8");
    return String(chunk);
  }
}
