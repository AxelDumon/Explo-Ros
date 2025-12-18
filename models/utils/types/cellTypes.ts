import { Document } from "./general.js";

////////////////// Types ////////////////////

// export type Cell = {
//   value: number;
// };

export type Cell = {
  x: number;
  y: number;
  value: number; // Between 0 and 100 indicating exploration state (probability of being occupied)
};

export type MapUpdate = {
    cells: Cell[]
}

export type CellDocument = Document &  { type: "cell", value: number };

////////////////// Functions ////////////////////

//// Type guards

export function isMapUpdate(obj: any): obj is MapUpdate {
    return (
        obj != null &&
        Array.isArray(obj.cells) &&
        obj.cells.every((cell : any) => isCell(cell))
    );
}

export function isCell(obj: any): obj is Cell {
  return (
    obj != null &&
    (typeof obj.x).toLowerCase() === 'number' &&
    (typeof obj.y).toLowerCase() === 'number' &&
    (typeof obj.value).toLowerCase() === 'number'
  );
}

//// Converters

/**
 * Converts a MapUpdate to a CellDocument array.
 * @param mapUpdate The MapUpdate to convert
 * @returns A CellDocument array
 */
export function fromMapUpdateToCellDocumentArray(
  mapUpdate: MapUpdate
): CellDocument[] {
  const cellTab: CellDocument[] = [];
  mapUpdate.cells.forEach((cell) => {
    cellTab.push({
      _id: `${cell.x},${cell.y}`,
      value: cell.value,
      type: "cell",
    } as CellDocument);
  });
  return cellTab;
}

/**
 * Converts a CellDocument to Cell.
 * @param cellDoc The CellDocument to convert
 * @returns A Cell object
 */
export function fromCellDocumentToCell(cellDoc: CellDocument): Cell {
  const coords = cellDoc._id.split(",");
  return {
    x: parseInt(coords[0]),
    y: parseInt(coords[1]),
    value: cellDoc.value,
  };
}

/**
 * Converts a CellDocument to a Uint8Array of length 3.
 * The first two elements are the coordinates extracted from the _id,
 * and the third element is the cell value.
 * @param cellDoc The CellDocument to convert
 * @returns A Uint8Array representing the cell [x, y, value]
 */
// export function fromCellDocumentToUint8(cellDoc: CellDocument): Uint8Array {
//   const tab = new Uint8Array(3);
//   const coords = cellDoc._id.split(",");
//   tab[0] = parseInt(coords[0]);
//   tab[1] = parseInt(coords[1]);
//   tab[2] = cellDoc.state;
//   return tab;
// }
