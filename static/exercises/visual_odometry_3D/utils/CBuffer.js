class CBuffer {
    constructor(n) {
        this._array = new Array(n);
        this._start = 0;
        this._end = 0;
        this.length = 0;
        this.size = n;
    }
    get(i) {
        if (i < 0) return this.get(i + this.length);
        if (i >= this.length) return this.get(i - this.length);
        return this._array[i];
    }
    push(item) {
        this._array[this._end] = item;
        this._end++;
        if (this.length < this._array.length) this.length++;
        if (this.length === this._array.length) this._start++;
        if (this._start === this._array.length) this._start = 0;
        if (this._end === this._array.length) this._end = 0;
    }
    getPoints() {
        const points = [];
        for (let i = this._start; i < this.length + this._start; i++) {
            const v = this.get(i);
            if (v !== undefined && v !== this.get(this._end))
                points.push(v);
        }
        return points;
    }
}