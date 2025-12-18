import { Cell } from "../utils/types/cellTypes.js";

export class World {
  width: number;
  cells: Uint8Array;

  constructor(width: number, height: number) {
    this.width = width;
    this.cells = new Uint8Array(width * height);
    this.cells.fill(0);
  }

  static defaultMap(): World {
    return new World(10, 10);
  }

  revealCells(undiscoveredNeighbors: Cell[]) {
    const revealedCells: Cell[] = [];
    undiscoveredNeighbors.forEach((cell) => {
      revealedCells.push({ x: cell.x, y: cell.y, value: this.valueOf(cell.x, cell.y) });
    });
    return revealedCells;
  }

  setCell(x: number, y: number, value: number): void {
    this.cells[y * this.width + x] = value;
  }

  getRandomEmptyCell() {
    const emptyCells: Cell[] = [];
    for (let y = 0; y < this.cells.length / this.width; y++) {
      for (let x = 0; x < this.width; x++) {
        if (this.valueOf(x, y) === 0) {
          emptyCells.push({ x, y, value: 0 });
        }
      }
    }
    if (emptyCells.length === 0) {
      throw new Error("No empty cells available");
    }
    return emptyCells[Math.floor(Math.random() * emptyCells.length)];
  }

  valueOf(x: number, y: number): number {
    return this.cells[y * this.width + x];
  }

}
